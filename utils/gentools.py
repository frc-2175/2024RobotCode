class GeneratorWrapper:
    def __init__(self, gen) -> None:
        self.done = False
        self.gen = gen
    def __iter__(self):
        return self
    def __next__(self):
        try:
            return next(self.gen)
        except StopIteration:
            self.done = True
            raise StopIteration
        
def doneable(func):
    def makeWrapped(*args, **kwargs) -> GeneratorWrapper:
        return GeneratorWrapper(func(*args, **kwargs))
    return makeWrapped

def resume(gen):
    return next(gen, None)