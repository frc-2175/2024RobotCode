def signedPower(input: float, power: float = 2) -> float:
    sign = 0
    if input > 0:
        sign = 1
    elif input < 0:
        sign = -1

    return sign * abs(input) ** power