import numpy as np

from .helper import Helpers
from .map import Map


class FastSLAM:
    """
    This class handles FastSLAM
    """

    def __init__(
        self,
        meas_cov,
        input_noises,
        nr_of_particles: int = 1000,
        threshold_distance=2,
        max_landmark_range=0,
        motion_model_type="velocity",
    ):
        """
        Init the FastSLAM1.0 algorithm. One particle exists of a state (x, y, theta), a weight and a map with landmarks.

        Args:
            - meas_cov, 2x2 np array  : The measurement covariance used in "observation predictions" and new landmark initialization
            - input_noises, 3x2 np array or 3x3 array : The noise sigma parameters on the input (so on v, omega)
            - nr_of_particles=1000, float : The amount of particles in the particle filter
            - threshold_distance=2, float : The threshold distance used in the detection of new landmarks.
                                            Observation is a new landmark if the distance to the closest landmark is larger than threshold_distance
            - max_landmark_distance=0, float : The maximal range of any landmark before it gets filtered out. Set to 0 to disable this filter
            - motion_model_type='velocity', string : What kind of motion model should be used to process a FastSLAM step
        """

        # Switch between velocity or position model
        if motion_model_type == "position":
            self.motion_model = self.__position_model__
        elif motion_model_type == "velocity":
            self.motion_model = self.__velocity_model__
        else:
            raise f"The motion model type {motion_model_type} is unsupported. Please choose 'velocity' or 'position'"

        # Parameters
        self.measurement_covariance = meas_cov
        self.input_noises = input_noises
        self.threshold_distance = threshold_distance
        self.max_landmark_range = max_landmark_range

        self.nr_of_particles = nr_of_particles

        # Internal state
        self.particle_states = np.zeros((nr_of_particles, 3), dtype=np.float64)
        self.particle_paths = [[] for i in range(nr_of_particles)]
        self.particle_weights = np.ones(nr_of_particles, dtype=np.float128)
        self.particle_map = Map(nr_of_particles)

        # The best particle is determined by the one with the highest weight. Just take the first one to start
        self.best_particle_index = 0

    def __position_model__(self, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Position based motion model.
        It directly edits self.particle_states by "sampling" a positional motion model based on the arguments.

        Args:
            - u: An np.ndarray containing the delta movement regarding position (delta distance, delta theta)
            - dt: the time between the previous "step" and the current one
        """

        dx0, dtheta0 = u

        variances = np.dot(
            self.input_noises,
            np.array([abs(dx0), abs(dtheta0)], dtype=np.float64),
        )

        # Much more efficient to sample the standard normal function!
        dx = dx0 + np.random.standard_normal(self.nr_of_particles) * variances[0]
        dtheta = (
            dtheta0 + np.random.standard_normal(self.nr_of_particles) * variances[1]
        )

        orig_thetas = self.particle_states[:, 2]

        self.particle_states[:, 0] += dx * np.cos(orig_thetas)
        self.particle_states[:, 1] += dx * np.sin(orig_thetas)
        self.particle_states[:, 2] += dtheta

        # Adding some extra theta noise in the end increases the DoF!
        theta_noise = np.random.standard_normal(self.nr_of_particles) * variances[2]
        self.particle_states[:, 2] += theta_noise * dt

        # Makes sure that the angles don't explode
        self.particle_states[:, 2] = Helpers.pi_2_pi(self.particle_states[:, 2])

    def __velocity_model__(self, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Velocity based motion model.
        It directly edits self.particle_states by "sampling" a velocity based motion model based on the arguments.

        Args:
            - u: An np.ndarray containing the delta movement regarding velocity (delta linear velocity, delta angular velocity)
            - dt: the time between the previous "step" and the current one
        """

        v0, w0 = u

        variances = np.dot(
            self.input_noises, np.array([abs(v0), abs(w0)], dtype=np.float64)
        )

        # Much more efficient to sample the standard normal function!
        v = v0 + np.random.standard_normal(self.nr_of_particles) * variances[0]
        w = w0 + np.random.standard_normal(self.nr_of_particles) * variances[1]

        pos_update_matrix = np.zeros((self.nr_of_particles, 3), dtype=np.float64)

        orig_thetas = self.particle_states[:, 2]

        # Make an exception when he drives straight
        if w0 < 0.01:
            pos_update_matrix[:, 0] = v * dt * np.cos(orig_thetas)
            pos_update_matrix[:, 1] = v * dt * np.sin(orig_thetas)
        else:
            pos_update_matrix[:, 0] = -v / w * np.sin(orig_thetas) + v / w * np.sin(
                orig_thetas + w * dt
            )
            pos_update_matrix[:, 1] = v / w * np.cos(orig_thetas) - v / w * np.cos(
                orig_thetas + w * dt
            )
            pos_update_matrix[:, 2] = w * dt

        self.particle_states += pos_update_matrix

        # Add some theta noise after sampling to add an extra DoF
        theta_noise = np.random.standard_normal(self.nr_of_particles) * variances[2]
        self.particle_states[:, 2] += theta_noise * dt

        # Normalize angle
        self.particle_states[:, 2] = Helpers.pi_2_pi(self.particle_states[:, 2])

    def sample_particles(self, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Samples all particle states based on a motion model "distribution".
        Basically tries to estimate how the car has moved with control inputs u

        Args:
            u:  2x1 control input (meaning differs on chosen motion model)
            dt: the time since the last state update
        """
        self.motion_model(u, dt)

    def low_variance_resampling(self):
        """
        This function resamples all particles ~ the particle weight.
        It does this efficiently by low variance resampling or also known as 'roulette resampling'

        It basically replaces 'bad' particles with low weights by better ones with higher weights
        """

        new_particle_indices = np.zeros(self.nr_of_particles, dtype=np.int0)

        # Put all weight together in one big cumsum, which represents a wheel
        weights = np.cumsum(self.particle_weights)

        # Generate 'spokes'
        spokes = (
            np.cumsum(np.ones(self.nr_of_particles) / self.nr_of_particles)
            - np.random.random() * 1 / self.nr_of_particles
        )
        spokes *= weights[-1]  # Normalization

        # Now do some kind of 'merge sort', where you only look at the area the spokes have landed on
        for i, spoke in enumerate(spokes):
            new_particle_indices[i] = (
                np.argmax(weights[new_particle_indices[i - 1] :] > spoke)
                + new_particle_indices[i - 1]
            )

        # Resample step
        self.particle_states = self.particle_states[new_particle_indices]
        self.particle_weights = self.particle_weights[new_particle_indices]
        self.particle_map.landmarks = self.particle_map.landmarks[new_particle_indices]

    def get_prediction(self):
        """
        Returns the best predictions so far

        Returns: A tuple of
            - the particle state, 1x3 np array
            - the particle map landmarks, A particle_amount x  2 x (3xN) np array, per landmark [[mu_x, sigma_xx, sigma_xy],
                                                                                                 [mu_y, sigma_yx, sigma_yy]]
            - the landmark classes (in the same order as the particle map landmarks)
            - the particle path, a list of (previous) particle states
            - All particle states
            - All particle weights
        """

        # Particle state is

        return (
            self.particle_states[self.best_particle_index],
            self.particle_map.get_landmarks(self.best_particle_index),
            self.particle_map.get_landmark_classes(self.best_particle_index),
            np.array(self.particle_paths[self.best_particle_index]),
            self.particle_states,
            self.particle_weights,
        )

    def predict_observation(self, landmark_pos, pose):
        """
        Makes a prediction about what observation should ideally be received based on a received landmark position and current pose

        Args:
            - landmark_pos: the landmark_pos to "predict" relative to the base_link frame
            - pose: the current pose estimate of the robot

        Returns:
            predicted (and expected) observation of the landmark_pos and the jacobian H
        """
        dx, dy = landmark_pos - pose[:2]

        expected_observation = Helpers.landmark_pos_to_observation(landmark_pos, pose)
        expected_range, expected_bearing = expected_observation

        # This jacobian is only the part corresponding to the landmark because the kalman filter only changes landmark mu and sigma !
        # See course for more information
        # [[dr / dlx, dr / dly],
        #  [db / dlx, db / dly]]
        H = (
            np.array(
                [
                    [dx, dy],
                    [-1 * dy / expected_range, dx / expected_range],
                ],
                dtype=np.float64,
            )
            / expected_range
        )

        return expected_observation, H

    def step(self, u, dt, observations, resting=False):
        """
        This function applies the fastSLAM algorithm. It is no problem when no observations are given (so only control input and odometry)

        Args:
          u: the control inputs
          dt: the time step between the previous step and this one
          observations: a 3xN np array of N observations. An observation is [radius, bearing, class (or tag)]
          resting: Should be set to True when the car isn't moving. This prevents the estimates from drifting when not moving.
        """

        #
        # PARTICLE SET SAMPLING
        #
        # Sample all particles based on the motion model and control inputs

        for k in range(self.nr_of_particles):
            self.particle_paths[k].append(self.particle_states[k])

        if not resting:
            self.sample_particles(u, dt)

        # If no observations, just stop already
        if len(observations) == 0:
            return

        self.particle_weights = np.ones(self.nr_of_particles, dtype=np.float128)

        # Now iterate over each observation
        for m, full_observation in enumerate(observations):

            observation = full_observation[:2]
            observation_class = full_observation[2]

            # Iterate over each particle
            for k in range(self.nr_of_particles):

                particle_state = self.particle_states[k]
                particle_map = self.particle_map

                #
                # LANDMARK ASSOCIATION
                #

                # Try to detect (based on the particles map) to which landmark this observations matches
                expected_landmark = Helpers.observation_to_landmark_pos(
                    observation, particle_state
                )
                (
                    observation_corresponding_distance,
                    observation_corresponding_id,
                ) = particle_map.search_corresponding_landmark_index(
                    k, expected_landmark
                )

                #
                # UPDATING PARTICLE MAP (KALMAN FILTERS)
                #

                # Check if feature has already been seen before or not
                if (
                    len(observation_corresponding_id) == 0
                    or observation_corresponding_distance[0] > self.threshold_distance
                ):

                    #
                    # LANDMARK INITIALIZATION
                    #

                    # New landmark!
                    landmark_mu = Helpers.observation_to_landmark_pos(
                        observation, particle_state
                    )  # Initialize mu

                    # Initialize EKF for this landmark and particle
                    # The expected observation in the beginning is just the 'observation' itself, because we don't know the landmark...
                    # Doesn't matter that much anyways

                    # Get the "expected observation" (which here is just the observation itself...) and the jacobian of the observation measurement
                    _, H = self.predict_observation(landmark_mu, particle_state)

                    Hinv = H ** (-1)
                    landmark_sigma = (
                        Hinv @ self.measurement_covariance
                    ) @ Hinv.T  # Initialize sigma

                    particle_map.add_landmark(
                        k,
                        np.hstack((landmark_mu.reshape((2, 1)), landmark_sigma)),
                        observation_class,
                        score=1,
                    )  # Add to map

                else:

                    #
                    # LANDMARK KALMAN FILTER UPDATE
                    #

                    # Landmark already exists and is matched to one on this map
                    matched_landmark_id = observation_corresponding_id[0]

                    # Calculate the expected_observation and the corresponding jacobian
                    expected_observation, H = self.predict_observation(
                        particle_map.get_landmark_mean(k, matched_landmark_id),
                        particle_state,
                    )

                    # Calculate Q (measurement covariance)
                    landmark_covariance = particle_map.get_landmark_covariance(
                        k, matched_landmark_id
                    )

                    Q = ((H @ landmark_covariance) @ H.T) + self.measurement_covariance
                    Qinv = np.linalg.inv(Q)

                    # Calculate kalman gain
                    K = (landmark_covariance @ H.T) @ Qinv

                    # Update landmark mean and covariance
                    observation_difference = observation - expected_observation

                    particle_map.set_landmark_mean(
                        k,
                        matched_landmark_id,
                        particle_map.get_landmark_mean(k, matched_landmark_id)
                        + (K @ observation_difference),
                    )

                    particle_map.set_landmark_covariance(
                        k,
                        matched_landmark_id,
                        (np.eye(2) - (K @ H)) @ landmark_covariance,
                    )

                    # Calculate the weight
                    factor = np.abs(2 * np.pi * np.linalg.det(Q)) ** (-1 / 2) * np.exp(
                        -1
                        / 2
                        * (((observation_difference.T @ Qinv) @ observation_difference))
                    )

                    # Reset the weight at the start of the procedure
                    if m == 0:
                        self.particle_weights[k] = 1

                    self.particle_weights[k] *= factor

                    # The landmark score keeps track of when a certain landmark has been "seen".
                    # In conjunction with some kind of predictor this can be used to filter out bad landmarks (landmarks with bad scores)
                    # Predictor: TODO
                    particle_map.increment_landmark_score(k, matched_landmark_id)

        #
        # RESAMPLING PARTICLE SET
        #
        # Resample particles
        self.low_variance_resampling()

        # Choose a new best particle
        self.best_particle_index = np.argmax(self.particle_weights)
