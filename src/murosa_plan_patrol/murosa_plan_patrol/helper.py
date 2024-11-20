def action_string_to_tuple(actionString):
    return tuple(actionString.split(','))

def action_tuple_to_string(actionTuple):
    return ','.join(actionTuple)
