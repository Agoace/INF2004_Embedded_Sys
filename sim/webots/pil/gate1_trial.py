"""Operator D-TRIAL identification only; not physical Gate1 acceptance."""
TRIAL_LABEL = 'TRIAL / NOT GATE 1 PASS'


def labels(trial):
    return ({'trial_clock_400_sync20': True, 'qualification': TRIAL_LABEL,
             'physical_gate1_pass': False} if trial else {})


def manifest():
    return {**labels(True), 'operator_decision': '2026-11-14 D-TRIAL',
            'scope': 'C1 only baseline plus explicit opt-in trial; not C2/C3',
            'clock_halfwidth_us': 400, 'sync_max_period_us': 20000,
            'unchanged_us': {'source_phase': 250, 'native_jitter': 250,
                             'availability': 3000, 'pipeline': 4000, 'expiry': 5000},
            'capture_policy': 'no early midpoint; qualified at start/end with same cones',
            'raw_artifacts': 'RAW UART/pcap bytes unchanged; retain this sidecar with external captures'}
