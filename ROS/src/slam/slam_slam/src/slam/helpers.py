import numpy as np
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped


def observations_to_range_bearings(observations: ObservationWithCovarianceArrayStamped):
    """
    Transforms ObservationWithCovarianceArrayStamped message to range_bearing messages
    Args:
        - observations: ObservationWithCovarianceArrayStamped, the message to transform

    Returns:
        A range-bearing representation of the message in the form of an (x, 3) numpy array ([range, bearing, class])
    """
    obs_bearings = np.zeros((len(observations.observations), 3))
    indices = np.zeros(len(observations.observations), dtype=bool)
    for i, obs_with_cov in enumerate(observations.observations):
        obs = obs_with_cov.observation
        
        d = np.sqrt(obs.location.x**2 + obs.location.y**2)

        indices[i] = True

        angle = pi_2_pi(np.arctan2(obs.location.y, obs.location.x))
        obs_bearings[i] = np.array([d, angle, obs.observation_class])

    return obs_bearings[indices, :]


def pi_2_pi(theta) -> float:
    """
    Returns a normalized angle between [-pi, pi]
    """
    return np.remainder(theta + np.pi, 2 * np.pi) - np.pi
