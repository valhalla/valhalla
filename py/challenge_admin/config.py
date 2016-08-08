
database_file = 'sqlite:///maproulette_tasks.sqlite'
geojson_file = 'maproulette_tasks.geojson'
url = 'http://localhost:9000'
api_key = '2-gkXTYPGqJBIW3mP/fADklG9Tgy3JHYb0eeM6y2rR8Tgo1bNy/r1cuWBiRYTik0dtimjcbLxI'
required_fields = ['name', 'instruction', 'id']
fields = ['name', 'parent', 'identifier', 'difficulty', 'description', 'blurb', 'instruction']
header = {'Content-Type': 'application/json', 'apiKey': api_key}
