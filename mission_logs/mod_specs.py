import json

specs_file = "src/patrol_mission_spec/data/robots_spec/robots_spec_inst_5r_20w_01.json"

with open(specs_file) as f:
  data = json.load(f)
  

cr_file = open("mission_logs/com_range.txt", "r")
cr_value = float(cr_file.read())
  
print(cr_value)

for r in data:
        data[r]["com_range"] = cr_value

with open(specs_file, 'w', encoding='utf-8') as outfile:        
        json.dump(data, outfile, ensure_ascii=False, indent=4)
