#!/usr/bin/env python3
"""Compile the actual B1 sections without the broken full-suite SMP stubs.

This is source extraction, not a replacement implementation. Reject additions
or calls inside a section that would invalidate the audited static bound.
"""
from pathlib import Path
import re
import sys


def clean(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)


def function(text, name):
    start = re.search(
        rf'^(?:static |EXPORT )?(?:void|bool|ER) {name}\([^;]*?\)\s*\{{',
        text, re.M)
    assert start, name
    brace = text.index('{', start.start())
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start.start():end]


def section(text, lock):
    match = re.search(
        rf'UINT intsts;\s*\(void\)ISpinLock\(&{lock}, &intsts\);'
        rf'.*?\(void\)ISpinUnlock\(&{lock}, intsts\);', text, re.S)
    assert match, lock
    body = match.group()
    calls = re.findall(r'\b([A-Za-z_]\w*)\s*\(', body)
    assert all(call in ('ISpinLock', 'ISpinUnlock', 'if')
               for call in calls), calls
    return body


def main():
    root = Path(sys.argv[1])
    # Only API definitions in the UP shim may remain. Check all source files,
    # including radio-off TEST and optional profiles, rather than one CPP view.
    for directory in ('app_program', 'lib/libwifi'):
        for path in (root / directory).rglob('*'):
            if path.suffix not in ('.c', '.h'):
                continue
            source = clean(path.read_text())
            plain = re.findall(r'\b(SpinLock|SpinTryLock|SpinUnlock)\s*\(',
                               source)
            if path == root / 'app_program/platform/smp_single_core.c':
                assert plain == ['SpinLock', 'SpinTryLock', 'SpinUnlock']
            else:
                assert not plain, str(path)
    print('PASS: no plain lock callers in app_program or libwifi')
    if len(sys.argv) == 2:
        return
    mqtt = clean((root / 'app_program/subsystem1/robot_mqtt.c').read_text())
    wifi = clean((root / 'lib/libwifi/sysdepend/pico_rp2040/'
                  'cyw43_utk.c').read_text())
    helpers = (
        'inc', 'metrics_state_set', 'metrics_subscribed_set',
        'metrics_subscribed_and_state', 'metrics_commands_enabled_set',
        'metrics_sonar_missed_add', 'metrics_sonar_sequence_set',
        'metrics_observed_priority_unset',
        'metrics_observed_priority_set_if_unset', 'metrics_hook_max_update')
    kernel = clean((root / 'kernel/tkernel/smp_lock.c').read_text())
    output = ['/* Generated from production sources. Do not edit. */',
              function(kernel, 'ISpinLock'),
              function(kernel, 'ISpinUnlock')]
    assert mqtt.count('ISpinLock(&metrics_lock,') == 11
    assert wifi.count('ISpinLock(&status_lock,') == 2
    for name in helpers:
        body = function(mqtt, name)
        section(body, 'metrics_lock')
        output.append(body)
    snapshot = section(function(mqtt, 'robot_network_metrics'), 'metrics_lock')
    output.append('static void snapshot_read(void)\n{\n'
                  '    rm_heartbeat_t snapshot;\n'
                  '    bool connected, subscribed_now, commands_now;\n' +
                  snapshot + '\n'
                  '    assert(snapshot.mqtt_state == metrics.mqtt_state);\n'
                  '    assert(connected == (metrics.mqtt_state >= 2u));\n'
                  '    assert(subscribed_now == subscribed);\n'
                  '    assert(commands_now == commands_enabled);\n}\n')
    publish = section(function(wifi, 'cyw43_utk_task'), 'status_lock')
    output.append('static void status_publish(void)\n{\n' + publish + '\n}')
    getter = function(wifi, 'cyw43_utk_get_status')
    section(getter, 'status_lock')
    output.append(getter)
    Path(sys.argv[2]).write_text('\n\n'.join(output) + '\n')


if __name__ == '__main__':
    main()
