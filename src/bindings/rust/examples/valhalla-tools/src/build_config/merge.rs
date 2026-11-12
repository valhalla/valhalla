//! The script's `merge_json`: an existing config keeps its values and gains the new keys.

use indexmap::IndexMap;

use super::json::Value;

/// Key paths, dot-joined, that a merge added or removed.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct MergeReport {
    pub added: Vec<String>,
    pub removed: Vec<String>,
}

impl MergeReport {
    /// The script's `print_report` text, for stderr.
    pub fn render(&self) -> String {
        let mut text = String::new();
        for (paths, title, mark) in [
            (&self.added, "Added (from new config)", '+'),
            (&self.removed, "Removed (not in new config)", '-'),
        ] {
            if paths.is_empty() {
                continue;
            }
            text.push_str(&format!("\n{title}:\n"));
            let mut sorted: Vec<&String> = paths.iter().collect();
            sorted.sort();
            for path in sorted {
                text.push_str(&format!("  {mark} {path}\n"));
            }
        }
        if self.added.is_empty() && self.removed.is_empty() {
            text.push_str("\nNo differences found.\n");
        }
        text
    }
}

/// Whether a key found only in the old config goes: `None` keeps every such key, an empty list
/// removes them all, otherwise only the listed paths and their children go.
pub fn should_remove_key(key_path: &str, remove_keys: Option<&[String]>) -> bool {
    match remove_keys {
        None => false,
        Some([]) => true,
        Some(paths) => paths.iter().any(|path| {
            key_path
                .strip_prefix(path.as_str())
                .is_some_and(|rest| rest.is_empty() || rest.starts_with('.'))
        }),
    }
}

/// Merges `new` into `old`: values of `old` win, keys of `old` keep their order, keys only in
/// `new` follow them; a non-dict `new` keeps `old`, a non-dict `old` takes `new`.
pub fn merge_json(
    old: &Value,
    new: &Value,
    report: &mut MergeReport,
    remove_keys: Option<&[String]>,
    path: &str,
) -> Value {
    let Value::Dict(new_entries) = new else {
        return old.clone();
    };
    let Value::Dict(old_entries) = old else {
        return new.clone();
    };
    let key_path = |key: &str| {
        if path.is_empty() {
            key.to_owned()
        } else {
            format!("{path}.{key}")
        }
    };
    let mut result = IndexMap::new();
    for (key, old_value) in old_entries {
        let key_path = key_path(key);
        if let Some(new_value) = new_entries.get(key) {
            let merged = merge_json(old_value, new_value, report, remove_keys, &key_path);
            result.insert(key.clone(), merged);
        } else if should_remove_key(&key_path, remove_keys) {
            report.removed.push(key_path);
        } else {
            result.insert(key.clone(), old_value.clone());
        }
    }
    for (key, new_value) in new_entries {
        if !old_entries.contains_key(key) {
            result.insert(key.clone(), new_value.clone());
            report.added.push(key_path(key));
        }
    }
    Value::Dict(result)
}
