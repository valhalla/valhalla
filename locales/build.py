#!/usr/bin/env python
import sys
import io
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
  print(d)
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
        perms = (dict(zip(inp.keys(), values)) for values in product(*inp.values()))
        for perm in perms:
            phrase = raw_phrase
            for token, replacement in perm.items():
                phrase = phrase.replace('<{}>'.format(token), replacement)
            examples.append(phrase)
    return examples

#build examples for each phrase using example_tokens in the yaml file
def attach_examples(json):
    instructions = json['instructions']
    for instruction in instructions:
        instructions[instruction]['example_phrases'] = {}
        for phrase_idx in instructions[instruction]['phrases']:
            phrase = instructions[instruction]['phrases']["{}".format(phrase_idx)]
            tokens = re.findall('\<(\w+)\>', phrase)
            example_phrases = []
            instructions[instruction]['example_phrases'][phrase_idx] = make_example_phrases(phrase, tokens, json['example_tokens'])

def main():
    #load each yaml file in the current directory
    for yaml_file in glob.glob("*.yml"):
        with io.open(yaml_file, encoding = 'utf-8', mode = 'r') as f:
            print("[LOCALES] Loading {}".format(yaml_file))
            #parse yaml as json
            y=yaml.load(f, Loader=yaml.FullLoader)

        lang = list(y.keys())[0]
        instructions = y[lang]
        #adds example_phrases to each instruction
        attach_examples(instructions)
        #TODO sort the json
        # sort_dict(None, instructions)
        #stringify json
        contents = json.dumps(instructions, ensure_ascii = False, indent = 2)
        #print the json to a file with the same name
        json_file=yaml_file.replace(".yml", ".json")
        with io.open(json_file, encoding = 'utf-8', mode = 'w') as f:
            f.write(contents)
            print("[LOCALES] Updated {}".format(json_file))
main()
