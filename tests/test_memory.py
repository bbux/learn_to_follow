import memory
import numpy as np

STATE_DIM = 1
ACTION_DIM = 2

def test_save_load():
    mem = memory.Memory(10, dims=2 * STATE_DIM + ACTION_DIM + 1)
    for i in range(100):
        s = i % 4
        a = [i % 3, i% 3]
        r = i % 2 -1
        s_ = s + 1
        mem.store_transition(s, a, r, s_)
    
    saved_data = mem.data
    mem.save("/tmp/test_mem_save")
    
    
    mem = memory.Memory(10, dims=2 * STATE_DIM + ACTION_DIM + 1)
    mem.load("/tmp/test_mem_save")
    
    assert np.array_equal(saved_data, mem.data)
    
    
