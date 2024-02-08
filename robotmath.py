def deadband(value, band= 0.1):
    result = 0
    if value> band:
        result = (value - band)/ (1-band)
    elif value < -band :
        result = (value + band)/ (1-band)
    return result

    

