markers = {
    1: "start",
    7: "radioactive",
    11: "explosives",
    21: "flammable",
    22: "non-flammable gas",
    41: "dangous when wet",
    42: "combustible",
    43: "flammable solid",
    51: "oxidizer",
    52: "orgnic peroxide",
    61: "posion",
    62: "inhalation hazard"
}

def functionA(id):
    if id in markers:
        return markers[id]
    return None