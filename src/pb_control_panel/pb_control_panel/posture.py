import math


class SentryPostureStateMachine:
    ATTACK = 1
    DEFENSE = 2
    MOBILE = 3
    ENHANCED_ATTACK = 4
    ENHANCED_DEFENSE = 5
    ENHANCED_MOBILE = 6

    SWITCH_COOLDOWN_SECONDS = 5.0
    BASE_POSTURE_SECONDS = 180.0
    ENHANCED_POSTURE_SECONDS = 15.0

    POSTURE_NAMES = {
        ATTACK: "attack",
        DEFENSE: "defense",
        MOBILE: "mobile",
        ENHANCED_ATTACK: "enhanced attack",
        ENHANCED_DEFENSE: "enhanced defense",
        ENHANCED_MOBILE: "enhanced mobile",
    }

    def __init__(self):
        self.reset_match()

    def reset_match(self):
        self.posture_id = self.MOBILE
        self.is_powered = False
        self.cooldown_remaining = 0.0
        self.base_remaining = {
            self.ATTACK: self.BASE_POSTURE_SECONDS,
            self.DEFENSE: self.BASE_POSTURE_SECONDS,
            self.MOBILE: self.BASE_POSTURE_SECONDS,
        }
        self.enhanced_remaining = {
            self.ATTACK: self.ENHANCED_POSTURE_SECONDS,
            self.DEFENSE: self.ENHANCED_POSTURE_SECONDS,
            self.MOBILE: self.ENHANCED_POSTURE_SECONDS,
        }
        self.last_request_accepted = True
        self.last_request_message = "posture state reset to mobile"

    @property
    def effective_posture(self):
        return self.posture_id + 3 if self.is_powered else self.posture_id

    @property
    def is_weakened(self):
        return self.base_remaining[self.posture_id] <= 0.0

    def request(self, posture, game_running):
        if posture not in self.POSTURE_NAMES:
            return self._reject("invalid posture {}".format(posture))
        if not game_running:
            return self._reject(
                "posture switches are accepted only while the match is running"
            )
        if posture == self.effective_posture:
            return self._accept(
                "already in {} posture".format(self.POSTURE_NAMES[posture])
            )
        if self.cooldown_remaining > 0.0:
            return self._reject(
                "switch cooldown has {:.1f} seconds remaining".format(
                    self.cooldown_remaining
                )
            )

        base_posture = posture if posture <= self.MOBILE else posture - 3
        powered = posture > self.MOBILE
        if powered and self.enhanced_remaining[base_posture] <= 0.0:
            return self._reject(
                "{} posture allowance is exhausted".format(self.POSTURE_NAMES[posture])
            )

        self.posture_id = base_posture
        self.is_powered = powered
        self.cooldown_remaining = self.SWITCH_COOLDOWN_SECONDS
        return self._accept(
            "switched to {} posture".format(self.POSTURE_NAMES[posture])
        )

    def advance(self, elapsed_seconds, game_running):
        if elapsed_seconds <= 0.0 or not game_running:
            return

        elapsed_seconds = float(elapsed_seconds)
        self.cooldown_remaining = max(0.0, self.cooldown_remaining - elapsed_seconds)
        self.base_remaining[self.posture_id] = max(
            0.0, self.base_remaining[self.posture_id] - elapsed_seconds
        )

        if not self.is_powered:
            return

        self.enhanced_remaining[self.posture_id] = max(
            0.0, self.enhanced_remaining[self.posture_id] - elapsed_seconds
        )
        if self.enhanced_remaining[self.posture_id] <= 0.0:
            posture_name = self.POSTURE_NAMES[self.posture_id]
            self.is_powered = False
            self.cooldown_remaining = self.SWITCH_COOLDOWN_SECONDS
            self.last_request_accepted = True
            self.last_request_message = (
                "enhanced allowance exhausted; returned to {} posture".format(
                    posture_name
                )
            )

    def fill_status_message(self, message):
        message.posture_id = self.posture_id
        message.is_weakened = self.is_weakened
        message.is_powered = self.is_powered
        message.attack_remaining_time = math.ceil(self.base_remaining[self.ATTACK])
        message.defense_remaining_time = math.ceil(self.base_remaining[self.DEFENSE])
        message.mobile_remaining_time = math.ceil(self.base_remaining[self.MOBILE])
        message.enhanced_attack_remaining_time = self.enhanced_remaining[self.ATTACK]
        message.enhanced_defense_remaining_time = self.enhanced_remaining[self.DEFENSE]
        message.enhanced_mobile_remaining_time = self.enhanced_remaining[self.MOBILE]
        message.switch_cooldown_remaining = self.cooldown_remaining
        return message

    def _accept(self, message):
        self.last_request_accepted = True
        self.last_request_message = message
        return True, message

    def _reject(self, message):
        self.last_request_accepted = False
        self.last_request_message = message
        return False, message
