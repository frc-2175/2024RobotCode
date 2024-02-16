import json
import time
from types import FrameType, BuiltinFunctionType
from typing import TextIO

class Profiler:
    def __init__(self) -> None:
        self.start = time.time_ns()
        self.end = 0
        self.events = []
        self.initialized = False
        self.frame_indices = {}
        self.frames = []
        self.frame_stack = []
        self.raw_events = []


    def profile_func(self, frame: FrameType, event, arg):
        t = time.time_ns()

        stack = []
        f = frame
        while f:
            stack.append(f)
            f = f.f_back

        self.raw_events.append({
            "t": t,
            "stack": [self.frame_dict(f) for f in stack],
            "event": event,
            "arg": str(arg),
        })

        if not self.initialized:
            # Add begin events for all frames in the stack
            frames: list[FrameType] = []
            f = frame
            while f:
                frames.append(f)
                f = f.f_back
            frames.reverse()
            t = time.time_ns()
            for f in frames:
                fid = self.pyframeid(f)
                self.events.append(("O", t, fid))
                self.frame_stack.append(fid)
            self.initialized = True

        if event == "call":
            e = "O"
            fid = self.pyframeid(frame)
        elif event == "c_call":
            e = "O"
            fid = self.cframeid(frame, arg)
        elif event == "return":
            e = "C"
            fid = self.pyframeid(frame)
        elif event == "c_return":
            e = "C"
            fid = self.cframeid(frame, arg)
        else:
            raise Exception(f"unknown event type {event}")

        if event == "call" or event == "c_call":
            self.frame_stack.append(fid)
        else:
            popped = self.frame_stack.pop()
            if popped != fid:
                print("EXPECTED", self.frames[fid])
                print("ACTUAL", self.frames[popped])
                raise Exception(f"expected to pop frame {fid} but popped frame {popped}")

        self.events.append((e, t, fid))
        self.end = t


    def pyframeid(self, f: FrameType) -> int:
        pf = (f.f_code.co_name, f.f_code.co_filename, f.f_code.co_firstlineno)
        if pf not in self.frame_indices:
            self.frames.append(pf)
            self.frame_indices[pf] = len(self.frames) - 1
        return self.frame_indices[pf]


    def cframeid(self, f: FrameType, c: BuiltinFunctionType) -> int:
        pf = (c.__name__, f.f_code.co_filename, f.f_lineno)
        if pf not in self.frame_indices:
            self.frames.append(pf)
            self.frame_indices[pf] = len(self.frames) - 1
        return self.frame_indices[pf]


    def finalize(self):
        while len(self.frame_stack) > 0:
            self.events.append(("C", self.end, self.frame_stack.pop()))


    def write_raw_events(self, f: TextIO):
        json.dump(self.raw_events, f)


    def write_speedscope(self, f: TextIO):
        self.finalize()

        f.write(r'{"$schema":"https://www.speedscope.app/file-format-schema.json","shared":{"frames":[')
        for i, pf in enumerate(self.frames):
            if i > 0:
                f.write(r',')
            f.write(r'{"name":')
            f.write(json.dumps(pf[0]))
            f.write(r',"file":')
            f.write(json.dumps(pf[1]))
            f.write(r'}')
        f.write(r']},"profiles":[{"type":"evented","name":"rioprofile","unit":"nanoseconds","startValue":')
        f.write(json.dumps(self.start))
        f.write(r',"endValue":')
        f.write(json.dumps(self.end))
        f.write(r',"events":[')
        for i, event in enumerate(self.events):
            if i > 0:
                f.write(r',')
            f.write(r'{"type":')
            f.write(json.dumps(event[0]))
            f.write(r',"at":')
            f.write(json.dumps(event[1]))
            f.write(r',"frame":')
            f.write(json.dumps(event[2]))
            f.write(r'}')
        f.write(r']}]}')


    def frame_dict(self, f: FrameType):
        return {
            "f_code": {
                "co_filename": f.f_code.co_filename,
                "co_firstlineno": f.f_code.co_firstlineno,
            },
            "f_lineno": f.f_lineno,
        }
