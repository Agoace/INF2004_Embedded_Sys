"""Operator D-TRIAL identification only; not physical Gate1 acceptance."""
TRIAL_LABEL = 'TRIAL / NOT GATE 1 PASS'


def labels(trial):
    return ({'trial_clock_400_sync20': True, 'qualification': TRIAL_LABEL,
             'physical_gate1_pass': False} if trial else {})


def manifest():
    return {**labels(True), 'operator_decision': '2026-11-14 D-TRIAL',
            'scope': 'C1 + C2 + C2b + C2c capture opportunities; NOT C3 queued logging',
            'capture_first': True, 'queued_logger': False,
            'pc_capture_revision': 'C1+C2+C2b+C2c',
            'capture_checks': ['loop-entry', 'post-RX/pre-SYNC', 'post-SYNC', 'post-TX/flush/pre-sleep'],
            'fresh_pre_tx_expiry_check': True,
            'max_captures_per_iteration': 1,
            'clock_halfwidth_us': 400, 'sync_max_period_us': 20000,
            'unchanged_us': {'source_phase': 250, 'native_jitter': 250,
                             'availability': 3000, 'pipeline': 4000, 'expiry': 5000},
            'capture_policy': 'no early midpoint; qualified at start/end with same cones',
            'raw_artifacts': 'RAW UART/pcap bytes unchanged; retain this sidecar with external captures'}
