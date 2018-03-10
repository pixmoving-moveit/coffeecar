import cantools
from pprint import pprint


dbc = cantools.db.load_file('example.dbc')


dbc.decode(0x10630000, message.data)

#print(dbc.messages)
