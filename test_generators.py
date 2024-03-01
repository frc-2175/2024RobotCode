def printHello():
    while True:
        print("hello")
        yield

def printWorld():
    while True:
        print("world")
        yield

def printHelloWorld():
    h = printHello()
    w = printWorld()
    while True:
        print("-----")
        next(h)
        next(w)
        yield

helloGenerator = printHello()
worldGenerator = printWorld()
helloWorldGenerator = printHelloWorld()

while True:
    # next(helloGenerator)
    # next(worldGenerator)
    next(helloWorldGenerator)
    print(":)")