"""Operator D-TRIAL identification only; not physical Gate1 acceptance."""
TRIAL_LABEL = 'TRIAL / NOT GATE 1 PASS'


P_MAX_US = 600
LATE_WINDOW_SLOTS = 1000
TRIAL_COMPLETIONS = ('TRIAL_COMPLETE', 'TRIAL_COMPLETE_WITH_TOLERATED_LATE')


def late_policy():
    return {'nominal_phase_us': 250, 'max_phase_us': P_MAX_US,
            'window_slots': LATE_WINDOW_SLOTS, 'max_late_in_window': 1,
            'consecutive_late_allowed': False}


class LateTolerance:
    """One late capture per rolling1000 slots, never a deadline/age waiver."""
    def __init__(self):
        self.last_late_slot = None
        self.count = 0

    def rejection(self, slot, phase):
        if phase > P_MAX_US:
            return 'source phase exceeds600us T1 cap'
        if self.last_late_slot is not None:
            distance = slot-self.last_late_slot
            if distance == 1:
                return 'consecutive late source captures'
            if distance < LATE_WINDOW_SLOTS:
                return 'more than one late capture in rolling1000 slots'
        return None

    def commit(self, slot):
        self.last_late_slot = slot
        self.count += 1


def labels(trial, tolerate_late=False):
    result = ({'trial_clock_400_sync20': True, 'qualification': TRIAL_LABEL,
               'physical_gate1_pass': False} if trial else {})
    if tolerate_late:
        if not trial: raise ValueError('late tolerance requires explicit clock400 trial')
        result.update(trial_tolerate_late=True, late_tolerance_policy=late_policy())
    return result


def manifest(tolerate_late=False):
    result = {**labels(True, tolerate_late), 'operator_decision': '2026-11-15 T1' if tolerate_late else '2026-11-14 D-TRIAL',
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
    if tolerate_late:
        result['unchanged_us'].pop('source_phase')
        result.update(nominal_source_phase_us=250, tolerated_source_phase_us=P_MAX_US)
    return result
