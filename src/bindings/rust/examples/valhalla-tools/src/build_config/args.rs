//! The script's command line, parsed the way Python 3.14's `argparse` parses it. clap only
//! renders the help and usage errors: its parser cannot classify arguments as argparse does.

use std::collections::HashMap;
use std::ffi::{OsStr, OsString};
use std::path::PathBuf;

use clap::{Arg, ArgAction, Command};

use super::json::Value;
use super::table::{DefaultValue, LEAVES, Literal, OptionalKind, endpoint};
use crate::python::{float_repr, is_decimal_digit, parse_float, parse_int, str_repr, strtobool};

/// The arguments given to the script, before any file is read.
#[derive(Debug, Default, PartialEq)]
pub struct Arguments {
    /// One per [`LEAVES`] entry, `None` where the option was not given.
    pub values: Vec<Option<Value>>,
    /// Paths need not be UTF-8, as with Python's surrogate escapes.
    pub merge_config: Option<PathBuf>,
    pub output: Option<PathBuf>,
    /// `--remove-old`: `None` when absent, empty when given without keys.
    pub remove_old: Option<Vec<String>>,
    pub report: bool,
}

/// What a command line asks for.
#[derive(Debug, PartialEq)]
pub enum Parsed {
    Help,
    Run(Arguments),
}

/// A usage error, with the same message as `argparse` after its `error: `.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum UsageError {
    #[error("ambiguous option: {option} could match {matches}")]
    Ambiguous { option: String, matches: String },
    #[error("argument {0}: expected one argument")]
    ExpectedOne(String),
    #[error("argument {0}: expected at least one argument")]
    ExpectedAtLeastOne(String),
    #[error("argument {action}: ignored explicit argument {}", str_repr(.value))]
    IgnoredExplicit { action: String, value: String },
    #[error("argument {action}: invalid {type_name} value: {}", str_repr(.value))]
    InvalidValue {
        action: String,
        type_name: &'static str,
        value: String,
    },
    #[error("unrecognized arguments: {0}")]
    Unrecognized(String),
    /// Not `argparse`'s: only `-m` and `-o` take values that are not UTF-8.
    #[error("argument {action}: value is not valid UTF-8: {}", str_repr(.value))]
    NotUtf8 { action: String, value: String },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Action {
    Help,
    Leaf(usize),
    MergeConfig,
    Output,
    RemoveOld,
    Report,
}

#[derive(Clone, Copy)]
enum Nargs {
    Zero,
    One,
    OneOrMore,
    ZeroOrMore,
}

impl Action {
    fn nargs(self) -> Nargs {
        match self {
            Self::Help | Self::Report => Nargs::Zero,
            Self::RemoveOld => Nargs::ZeroOrMore,
            Self::Leaf(index) => match LEAVES[index].default {
                DefaultValue::Optional(OptionalKind::List) => Nargs::OneOrMore,
                _ => Nargs::One,
            },
            Self::MergeConfig | Self::Output => Nargs::One,
        }
    }

    /// How `argparse` names the action in its errors.
    fn name(self) -> String {
        match self {
            Self::Help => "-h/--help".to_owned(),
            Self::Leaf(index) => format!("--{}", LEAVES[index].option),
            Self::MergeConfig => "-m/--merge-config".to_owned(),
            Self::Output => "-o/--output".to_owned(),
            Self::RemoveOld => "-r/--remove-old".to_owned(),
            Self::Report => "-p/--report".to_owned(),
        }
    }
}

/// One reading of an option-like argument: `argparse`'s `(action, option_string, sep,
/// explicit_arg)`, where only a `=` separator counts.
#[derive(Clone, Debug)]
struct Match {
    /// `None` for an unknown option.
    action: Option<Action>,
    option: String,
    equals: bool,
    explicit: Option<String>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Pattern {
    Value,
    Option,
    DoubleDash,
}

struct Options {
    /// In `argparse`'s registration order, which its ambiguity errors follow.
    ordered: Vec<(String, Action)>,
    exact: HashMap<String, Action>,
}

impl Options {
    fn new() -> Self {
        let mut ordered = vec![
            ("-h".to_owned(), Action::Help),
            ("--help".to_owned(), Action::Help),
        ];
        ordered.extend(
            LEAVES
                .iter()
                .enumerate()
                .map(|(index, leaf)| (format!("--{}", leaf.option), Action::Leaf(index))),
        );
        for (short, long, action) in [
            ("-m", "--merge-config", Action::MergeConfig),
            ("-o", "--output", Action::Output),
            ("-r", "--remove-old", Action::RemoveOld),
            ("-p", "--report", Action::Report),
        ] {
            ordered.push((short.to_owned(), action));
            ordered.push((long.to_owned(), action));
        }
        let exact = ordered.iter().cloned().collect();
        Self { ordered, exact }
    }

    fn exact_match(&self, option: &str, equals: bool, explicit: Option<&str>) -> Option<Match> {
        self.exact.get(option).map(|&action| Match {
            action: Some(action),
            option: option.to_owned(),
            equals,
            explicit: explicit.map(str::to_owned),
        })
    }

    /// `_parse_optional`: `None` when the argument is a value.
    fn classify(&self, arg: &str) -> Option<Vec<Match>> {
        if !arg.starts_with('-') {
            return None;
        }
        if let Some(found) = self.exact_match(arg, false, None) {
            return Some(vec![found]);
        }
        if arg.chars().count() == 1 {
            return None;
        }
        if let Some((option, explicit)) = arg.split_once('=')
            && let Some(found) = self.exact_match(option, true, Some(explicit))
        {
            return Some(vec![found]);
        }
        let matches = self.prefix_matches(arg);
        if !matches.is_empty() {
            return Some(matches);
        }
        let mut chars = arg.chars().skip(1);
        let negative_number = match chars.next() {
            Some('.') => chars.next().is_some_and(is_decimal_digit),
            Some(c) => is_decimal_digit(c),
            None => false,
        };
        if negative_number || arg.contains(' ') {
            return None;
        }
        Some(vec![Match {
            action: None,
            option: arg.to_owned(),
            equals: false,
            explicit: None,
        }])
    }

    /// `_get_option_tuples`: the options `arg` abbreviates, or whose short form it starts with.
    fn prefix_matches(&self, arg: &str) -> Vec<Match> {
        let (prefix, equals, explicit) = match arg.split_once('=') {
            Some((prefix, explicit)) => (prefix, true, Some(explicit)),
            None => (arg, false, None),
        };
        let reading = |action, option: &str, equals, explicit: Option<&str>| Match {
            action: Some(action),
            option: option.to_owned(),
            equals,
            explicit: explicit.map(str::to_owned),
        };
        if arg.starts_with("--") {
            return self
                .ordered
                .iter()
                .filter(|(option, _)| option.starts_with(prefix))
                .map(|(option, action)| reading(*action, option, equals, explicit))
                .collect();
        }
        let split = arg
            .char_indices()
            .nth(2)
            .map_or(arg.len(), |(index, _)| index);
        let (short, short_explicit) = arg.split_at(split);
        self.ordered
            .iter()
            .filter_map(|(option, action)| {
                if option == short {
                    Some(reading(*action, option, false, Some(short_explicit)))
                } else if option.starts_with(prefix) {
                    Some(reading(*action, option, equals, explicit))
                } else {
                    None
                }
            })
            .collect()
    }
}

/// `_match_argument`: how many of the arguments at the start of `patterns` the action takes.
fn match_argument(action: Action, patterns: &[Pattern]) -> Result<usize, UsageError> {
    let values = patterns
        .iter()
        .take_while(|&&p| p == Pattern::Value)
        .count();
    match action.nargs() {
        Nargs::Zero => Ok(0),
        Nargs::One if values >= 1 => Ok(1),
        Nargs::One => Err(UsageError::ExpectedOne(action.name())),
        Nargs::OneOrMore if values >= 1 => Ok(values),
        Nargs::OneOrMore => Err(UsageError::ExpectedAtLeastOne(action.name())),
        Nargs::ZeroOrMore => Ok(values),
    }
}

/// A value: the argument it comes from, and its text, a suffix of that argument's.
struct Taken {
    arg: usize,
    text: String,
}

/// Parses the arguments, program name excluded.
pub fn parse_args(args: &[OsString]) -> Result<Parsed, UsageError> {
    let texts: Vec<String> = args
        .iter()
        .map(|arg| arg.to_string_lossy().into_owned())
        .collect();
    let options = Options::new();
    let mut patterns = Vec::with_capacity(args.len());
    let mut readings: Vec<Option<Vec<Match>>> = Vec::with_capacity(args.len());
    let mut after_double_dash = false;
    for text in &texts {
        if after_double_dash {
            patterns.push(Pattern::Value);
            readings.push(None);
        } else if text == "--" {
            after_double_dash = true;
            patterns.push(Pattern::DoubleDash);
            readings.push(None);
        } else {
            let reading = options.classify(text);
            patterns.push(if reading.is_some() {
                Pattern::Option
            } else {
                Pattern::Value
            });
            readings.push(reading);
        }
    }

    let mut parser = ArgParser {
        args,
        texts: &texts,
        options: &options,
        patterns: &patterns,
        extras: Vec::new(),
        arguments: Arguments {
            values: vec![None; LEAVES.len()],
            ..Arguments::default()
        },
    };
    let mut index = 0;
    while let Some(next) = (index..texts.len()).find(|&i| readings[i].is_some()) {
        parser.extras.extend(texts[index..next].iter().cloned());
        let matches = readings[next].as_deref().unwrap_or_default();
        match parser.consume_optional(next, matches)? {
            Some(stop) => index = stop,
            None => return Ok(Parsed::Help),
        }
    }
    parser.extras.extend(texts[index..].iter().cloned());
    if !parser.extras.is_empty() {
        return Err(UsageError::Unrecognized(parser.extras.join(" ")));
    }
    Ok(Parsed::Run(parser.arguments))
}

/// The end of `arg` from byte `offset`. The bytes before it spell options, so they are ASCII and
/// `offset` is the same in `arg` and in its lossy text.
#[cfg(unix)]
fn os_suffix(arg: &OsStr, offset: usize) -> Option<OsString> {
    use std::os::unix::ffi::OsStrExt;
    let bytes = arg.as_bytes().get(offset..)?;
    Some(OsStr::from_bytes(bytes).to_owned())
}

#[cfg(not(unix))]
fn os_suffix(arg: &OsStr, offset: usize) -> Option<OsString> {
    arg.to_str()?.get(offset..).map(OsString::from)
}

struct ArgParser<'a> {
    args: &'a [OsString],
    /// The arguments as text, invalid UTF-8 replaced.
    texts: &'a [String],
    options: &'a Options,
    patterns: &'a [Pattern],
    extras: Vec<String>,
    arguments: Arguments,
}

impl ArgParser<'_> {
    /// `consume_optional`: takes the option at `index`, which may chain single-dash flags, and
    /// returns the index after its values, or `None` once help was asked for.
    fn consume_optional(
        &mut self,
        index: usize,
        matches: &[Match],
    ) -> Result<Option<usize>, UsageError> {
        let [found] = matches else {
            let options: Vec<&str> = matches.iter().map(|m| m.option.as_str()).collect();
            return Err(UsageError::Ambiguous {
                option: self.texts[index].clone(),
                matches: options.join(", "),
            });
        };
        let Match {
            mut action,
            mut option,
            mut equals,
            mut explicit,
        } = found.clone();
        let mut taken: Vec<(Action, Vec<Taken>)> = Vec::new();
        let stop = loop {
            let Some(current) = action else {
                self.extras.push(self.texts[index].clone());
                return Ok(Some(index + 1));
            };
            let Some(value) = explicit.take() else {
                let count = match_argument(current, &self.patterns[index + 1..])?;
                let values = (index + 1..index + 1 + count).map(|arg| Taken {
                    arg,
                    text: self.texts[arg].clone(),
                });
                taken.push((current, values.collect()));
                break index + 1 + count;
            };
            let count = match_argument(current, &[Pattern::Value])?;
            let ignored = || UsageError::IgnoredExplicit {
                action: current.name(),
                value: value.clone(),
            };
            if count == 1 {
                taken.push((
                    current,
                    vec![Taken {
                        arg: index,
                        text: value,
                    }],
                ));
                break index + 1;
            }
            let Some(first) = value.chars().next() else {
                return Err(ignored());
            };
            if option.starts_with("--") || equals || first == '-' {
                return Err(ignored());
            }
            // A flag written in a cluster such as `-pr`: the rest is more single-dash options.
            taken.push((current, Vec::new()));
            let next = format!("-{first}");
            let rest = &value[first.len_utf8()..];
            let Some(&next_action) = self.options.exact.get(&next) else {
                self.extras.push(format!("-{value}"));
                break index + 1;
            };
            action = Some(next_action);
            option = next;
            (equals, explicit) = match rest.strip_prefix('=') {
                _ if rest.is_empty() => (false, None),
                Some(after) => (true, Some(after.to_owned())),
                None => (false, Some(rest.to_owned())),
            };
        };
        for (action, values) in taken {
            if !self.take_action(action, values)? {
                return Ok(None);
            }
        }
        Ok(Some(stop))
    }

    /// Stores the values of an action; `false` when the action is help.
    fn take_action(&mut self, action: Action, values: Vec<Taken>) -> Result<bool, UsageError> {
        match action {
            Action::Help => return Ok(false),
            Action::Report => self.arguments.report = true,
            Action::MergeConfig => self.arguments.merge_config = Some(self.path(action, values)?),
            Action::Output => self.arguments.output = Some(self.path(action, values)?),
            Action::RemoveOld => {
                let keys = values.into_iter().map(|value| self.utf8(action, value));
                self.arguments.remove_old = Some(keys.collect::<Result<_, _>>()?);
            }
            Action::Leaf(index) => {
                let texts = values.into_iter().map(|value| self.utf8(action, value));
                let texts = texts.collect::<Result<Vec<_>, _>>()?;
                self.arguments.values[index] =
                    Some(convert(action, &LEAVES[index].default, texts)?);
            }
        }
        Ok(true)
    }

    fn utf8(&self, action: Action, value: Taken) -> Result<String, UsageError> {
        if self.args[value.arg].to_str().is_some() {
            Ok(value.text)
        } else {
            Err(UsageError::NotUtf8 {
                action: action.name(),
                value: value.text,
            })
        }
    }

    /// The single value of `-m` or `-o`, as given.
    fn path(&self, action: Action, values: Vec<Taken>) -> Result<PathBuf, UsageError> {
        let Some(value) = values.into_iter().next() else {
            return Err(UsageError::ExpectedOne(action.name()));
        };
        let offset = self.texts[value.arg].len() - value.text.len();
        match os_suffix(&self.args[value.arg], offset) {
            Some(path) => Ok(PathBuf::from(path)),
            None => Err(UsageError::NotUtf8 {
                action: action.name(),
                value: value.text,
            }),
        }
    }
}

/// The script's `type` for a leaf, applied to its values: several only for `Optional(list)`,
/// which keeps them as they are, a single one otherwise.
fn convert(
    action: Action,
    default: &DefaultValue,
    texts: Vec<String>,
) -> Result<Value, UsageError> {
    let invalid = |type_name, text: String| UsageError::InvalidValue {
        action: action.name(),
        type_name,
        value: text,
    };
    let single = |texts: Vec<String>| texts.into_iter().next_back().unwrap_or_default();
    match default {
        DefaultValue::Optional(OptionalKind::List) => {
            Ok(Value::List(texts.into_iter().map(Value::Str).collect()))
        }
        DefaultValue::Optional(OptionalKind::Int) | DefaultValue::Value(Literal::Int(_)) => {
            let text = single(texts);
            match parse_int(&text) {
                Some(digits) => Ok(Value::Int(digits)),
                None => Err(invalid("int", text)),
            }
        }
        DefaultValue::Optional(OptionalKind::Float) | DefaultValue::Value(Literal::Float(_)) => {
            let text = single(texts);
            match parse_float(&text) {
                Some(float) => Ok(Value::Float(float)),
                None => Err(invalid("float", text)),
            }
        }
        DefaultValue::Optional(OptionalKind::Str)
        | DefaultValue::Value(Literal::Str(_) | Literal::Endpoint { .. }) => {
            Ok(Value::Str(single(texts)))
        }
        // The script's `Optional(bool)` uses `bool(text)`, true unless empty, and its other bool
        // leaves a `strtobool` lambda; here every bool leaf uses `strtobool`.
        DefaultValue::Optional(OptionalKind::Bool) | DefaultValue::Value(Literal::Bool(_)) => {
            let text = single(texts);
            match strtobool(&text) {
                Some(flag) => Ok(Value::Bool(flag)),
                None => Err(invalid("<lambda>", text)),
            }
        }
        DefaultValue::Value(Literal::List(_)) => Ok(Value::List(
            single(texts)
                .split(',')
                .map(|item| Value::Str(item.to_owned()))
                .collect(),
        )),
        // `type(None)` refuses any value.
        DefaultValue::Value(Literal::Null) => Err(invalid("NoneType", single(texts))),
    }
}

/// The command for `--help` and usage errors, with one option per config leaf.
pub fn command() -> Command {
    let mut command = Command::new("valhalla_build_config")
        .about("Generate valhalla configuration")
        // Option names are up to 90 characters: help beside them would not fit.
        .next_line_help(true);
    for leaf in LEAVES {
        let mut arg = Arg::new(leaf.option)
            .long(leaf.option)
            .value_name(value_name(&leaf.default))
            .help(leaf.help);
        arg = match &leaf.default {
            DefaultValue::Optional(OptionalKind::List) => arg.num_args(1..),
            DefaultValue::Optional(_) => arg,
            DefaultValue::Value(literal) => arg.default_value(display(literal)),
        };
        command = command.arg(arg);
    }
    command
        .arg(
            Arg::new("merge-config")
                .short('m')
                .long("merge-config")
                .value_name("PATH")
                .help(
                    "Path to existing config file to merge with. Existing values are preserved, \
                     new keys are added.",
                ),
        )
        .arg(
            Arg::new("output")
                .short('o')
                .long("output")
                .value_name("PATH")
                .help("Output file path (default: stdout)"),
        )
        .arg(
            Arg::new("remove-old")
                .short('r')
                .long("remove-old")
                .value_name("KEY")
                .num_args(0..)
                .help(
                    "When merging, remove old keys. Without arguments, removes all keys not in \
                     new config. With arguments (JSON dot notation, e.g. 'mjolnir.logging'), \
                     removes only those keys.",
                ),
        )
        .arg(
            Arg::new("report")
                .short('p')
                .long("report")
                .action(ArgAction::SetTrue)
                .help("When merging, print the merge report."),
        )
}

/// The kind of value, rather than argparse's upper-cased path: it keeps `--help` within 80
/// columns.
fn value_name(default: &DefaultValue) -> &'static str {
    match default {
        DefaultValue::Optional(OptionalKind::Int) | DefaultValue::Value(Literal::Int(_)) => "INT",
        DefaultValue::Optional(OptionalKind::Float) | DefaultValue::Value(Literal::Float(_)) => {
            "FLOAT"
        }
        DefaultValue::Optional(OptionalKind::Bool) | DefaultValue::Value(Literal::Bool(_)) => {
            "BOOL"
        }
        DefaultValue::Value(Literal::List(_)) => "A,B,...",
        DefaultValue::Optional(OptionalKind::List) => "VALUE",
        DefaultValue::Optional(OptionalKind::Str)
        | DefaultValue::Value(Literal::Str(_) | Literal::Endpoint { .. } | Literal::Null) => "TEXT",
    }
}

/// A default as it would be typed on the command line.
fn display(literal: &Literal) -> String {
    match literal {
        Literal::Null => "None".to_owned(),
        Literal::Bool(flag) => if *flag { "True" } else { "False" }.to_owned(),
        Literal::Int(digits) => (*digits).to_owned(),
        Literal::Float(float) => float_repr(*float),
        Literal::Str(text) => (*text).to_owned(),
        Literal::List(items) => items.iter().map(display).collect::<Vec<_>>().join(","),
        Literal::Endpoint { ipc, port } => endpoint(ipc, port, cfg!(windows)),
    }
}
