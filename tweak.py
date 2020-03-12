import utils
import tf
import copy

T_identity = [[1e-10, 1e-10, 1e-10], [1e-10, 1e-10, 1e-10, 1.0]]

def tweak(T_tool_to_obj_seq, T_tweak_seq):
    n = len(T_tweak_seq)
    assert len(T_tool_to_obj_seq) == n

    T_Gtweaked_to_B_seq = []

    T_cached = T_identity
    for i in range(n):
        if i == 0:
            T_G_to_pre = T_tool_to_obj_seq[i]
        else:
            T_G_to_obj = T_tool_to_obj_seq[i]
            T_pre_to_obj = T_tool_to_obj_seq[i-1]
            T_obj_to_pre = utils.invert_tf(T_pre_to_obj)

            T_G_to_pre = utils.convert(
                    T_G_to_obj,
                    T_obj_to_pre)

        T_tweak = T_tweak_seq[i]
        T_Gtweaked_to_G = utils.convert(T_tweak, T_G_to_pre)
        T_Gtweaked_to_B = utils.convert(T_Gtweaked_to_G, T_cached)
        T_Gtweaked_to_B_seq.append(T_Gtweaked_to_B)
        T_cached = copy.copy(T_Gtweaked_to_B)

    return T_Gtweaked_to_B_seq

def debug_tweak_rule(T_tool_to_obj_seq, param):
    n = len(T_tool_to_obj_seq)
    T_Gtweaked_to_B_seq = tweak(T_tool_to_obj_seq, [T_identity for i in range(n)])
    return T_Gtweaked_to_B_seq

def no_tweak_rule(T_tool_to_obj_seq, param):
    return T_tool_to_obj_seq

def simple_tweak_rule(T_tool_to_obj_seq, param):
    n = len(T_tool_to_obj_seq)
    #T_Gtweaked_to_B_seq = tweak(T_tool_to_obj_seq, [T_identity for i in range(n)])
    Q_rolled = lambda angle: tf.transformations.quaternion_from_euler(0, angle, 0)
    T_tweaks = [[[0.0, 0, 0], Q_rolled(param)]]
    for i in range(n-1):
        T_tweaks.append(T_identity)
    T_Gtweaked_to_B_seq = tweak(T_tool_to_obj_seq, T_tweaks)
    return T_Gtweaked_to_B_seq
