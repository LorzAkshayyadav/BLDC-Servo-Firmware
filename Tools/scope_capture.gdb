# =============================================================================
# scope_capture.gdb  --  pull a frozen scope buffer off the target over SWD.
#
#   (gdb) source Tools/scope_capture.gdb
#   (gdb) scope_capture
#
# Then:  python Tools/scope_plot.py
#
# WHY A GDB SCRIPT RATHER THAN FIRMWARE
#   At stage 4 there is no fieldbus, no object dictionary and no console
#   command handler -- but there is a debugger, and g_buf is a plain static
#   array at a fixed address.  This is the entire readout path until stage 7,
#   and it needs no target code at all.
#
#   Reading 250 KB over SWD takes a few seconds.  That is fine: the buffer is
#   frozen and the drive is stopped, which is exactly when you read it.
# =============================================================================

define scope_capture
    set pagination off

    # The control state is a file-static in scope_log.c, hence the qualifier.
    set logging file scope_meta.txt
    set logging overwrite on
    set logging enabled on
    printf "write_index %u\n", 'scope_log.c'::g.write_index
    printf "wrapped %u\n",     'scope_log.c'::g.wrapped
    printf "armed %u\n",       'scope_log.c'::g.armed
    printf "decimation %u\n",  'scope_log.c'::g.decimation
    printf "source %u %u %u %u %u %u %u %u\n", \
        'scope_log.c'::g.source[0], 'scope_log.c'::g.source[1], \
        'scope_log.c'::g.source[2], 'scope_log.c'::g.source[3], \
        'scope_log.c'::g.source[4], 'scope_log.c'::g.source[5], \
        'scope_log.c'::g.source[6], 'scope_log.c'::g.source[7]
    set logging enabled off

    dump binary memory scope.bin \
        &'scope_log.c'::g_buf[0][0] &'scope_log.c'::g_buf[8000][0]

    printf "captured -> scope.bin + scope_meta.txt\n"
end

document scope_capture
Dump the frozen scope buffer and its metadata to scope.bin / scope_meta.txt.
Halt the target first. Works whether the buffer was frozen by a fault or by
calling scope_log_freeze() from the debugger.
end
