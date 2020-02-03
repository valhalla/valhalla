#!/usr/bin/env python
import sys
import glob
import json
import re
import yaml
from collections import OrderedDict
from itertools import product

#if its a number sort by the number, if its one of the top level ones use that
#its tags if its parent isnt a number or a top listed one
#otherwise its an instruction so just sort those alphabetically
order = ['posix_locale', 'aliases', 'instructions', 'phrases','SPOT_FOR_TAGS','example_phrases']
def sort_order(parent, k):
  try:
    return int(k)
  except:
    try:
      return order.index(k)
    except:
      pass
  return order.index('SPOT_FOR_TAGS') if parent and not parent.isdigit() and parent not in order else k

#recursively turn a dict into an ordered dict (where insert order is preserved)
#and while doing so sort each subdict using the sorter above
def sort_dict(parent, d):
  ordered = OrderedDict()
  for k, v in sorted(sorted(d.items()), key = lambda kv: sort_order(parent, kv[0])):
    if isinstance(v, dict):
      ordered[k] = sort_dict(k, v)
    else:
      ordered[k] = v
  return ordered

#every combination of every token is used to generate every permutation of the example phrases
def make_example_phrases(raw_phrase, tokens, examples_tokens):
    examples = []
    if len(tokens) == 0:
        examples.append(raw_phrase)
    else:
        inp = {}
        for token in tokens:
            inp[token] = examples_tokens[token]
        permutations = (dict(zip(inp.keys(), values)) for values in product(*inp.values()))
        for perm in permutations:
            phrase = raw_phrase
            for token, replacement in perm.items():
                phrase = phrase.replace('<{}>'.format(token), str(replacement))
            examples.append(phrase)
    return examples

#build examples for each phrase using example_tokens in the yaml file
def attach_examples(json):
    instructions = json['instructions']
    for instruction_type in instructions:
        instructions[instruction_type]['example_phrases'] = {}
        for phrase_idx, phrase in instructions[instruction_type]['phrases'].items():
            tokens = re.findall('\<(\w+)\>', phrase)
            instructions[instruction_type]['example_phrases'][phrase_idx] = make_example_phrases(phrase, tokens, json['example_tokens'])


#create initial yaml files for each json language file starting with an english yaml file
#only useful for creating the initial yaml files, just keeping this function for posterity
def copy_en_yaml():
    for source_lang_f in glob.glob("*.json"):
        if source_lang_f == 'en-US.json':
            continue

        with open(source_lang_f, encoding = 'utf-8', mode = 'r') as f:
            source_lang=json.load(f, encoding = 'utf-8')

        with open("en-US.yml", encoding = 'utf-8', mode = 'r') as f:
            target_dict=yaml.load(f,  Loader=yaml.FullLoader)

        for instruction_type in target_dict['en']['instructions']:
            for phrase_idx in target_dict['en']['instructions'][instruction_type]['phrases']:
                target_dict['en']['instructions'][instruction_type]['phrases'][phrase_idx] = source_lang['instructions'][instruction_type]['phrases'][phrase_idx]

        yml_file=source_lang_f.replace(".json", ".yml")
        with open(yml_file, encoding = 'utf-8', mode = 'w') as f:
            f.write(yaml.dump(target_dict, allow_unicode=True, indent = 4))


#compile yaml language files to expected json format
def main():
    #load each yaml file in the current directory
    for yaml_file in glob.glob("*.yml"):
        with open(yaml_file, encoding = 'utf-8', mode = 'r') as f:
            print("[LOCALES] Loading {}".format(yaml_file))
            #parse yaml as json
            y=yaml.load(f, Loader=yaml.FullLoader)

        #discard the top level language key
        lang = list(y.keys())[0]
        inner_json = y[lang]

        #adds example_phrases to each instruction
        attach_examples(inner_json)

        #filter out top level keys to just the ones we need
        keep_keys = ["posix_locale", "aliases", "instructions"]
        output_json = {}
        for k in inner_json.keys():
          if k in keep_keys:
            output_json[k] = inner_json[k]

        #sort the json
        sort_dict(None, output_json)

        #stringify json
        contents = json.dumps(output_json, ensure_ascii = False, indent = 2)

        #print the json to a file with the same name
        json_file=yaml_file.replace(".yml", ".json")
        with open(json_file, encoding = 'utf-8', mode = 'w') as f:
            f.write(contents + "\n")
            print("[LOCALES] Updated {}".format(json_file))

if __name__ == '__main__':
    main()
