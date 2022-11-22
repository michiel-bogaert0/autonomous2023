from collections import deque

import numpy as np
import scipy.spatial


class Map:
    def __init__(self, amount, expected_nr_of_landmarks=100):
        """
        A helper class to contain a set of landmarks.

        A landmark has dimension 2x3. So self.landmarks can be represented as a amount x 2 x (3xexpected_nr_of_landmarks) matrix
        Per landmark:
          [[mu_x, sigma_xx, sigma_xy],
           [mu_y, sigma_yx, sigma_yy]]

        The 'map' is actually a multi-dimensional array that keeps track of 'amount' independent maps, each with a set of x,y cones

        Args:
          - amount: the amount of independent maps
          - expected_nr_of_landmarks: is used to determine the amount of space to reserve in memory. Speeds things up.
        """

        self.datatype = np.float64

        self.amount = amount
        self.expected_nr_of_landmarks = expected_nr_of_landmarks
        self.supported_nr_of_landmarks = expected_nr_of_landmarks

        # Initial allocation
        self.landmarks = np.empty(
            (amount, 2, 3 * expected_nr_of_landmarks), dtype=self.datatype
        )
        self.sizes = np.zeros(amount, dtype=int)
        self.scores = np.zeros((amount, expected_nr_of_landmarks), dtype=int)
        self.landmark_classes = np.zeros((amount, expected_nr_of_landmarks), dtype=int)

        # KDTree for caching purposes
        self.kdtree = [None for i in range(amount)]

        self.map_id = 0

    def get_kdtree(self):
        return self.kdtree[self.map_id]

    def expand_map(self):
        """
        This method expands the map by enlarging the map array and other variables.
        It does this by just adding expected_nr_of_landmarks cones to the map
        """

        # First expand landmarks
        new_landmarks = np.empty(
            (self.amount, 2, 3 * self.expected_nr_of_landmarks), dtype=self.datatype
        )
        self.landmarks = np.concatenate((self.landmarks, new_landmarks), axis=2)

        # Then scores
        new_scores = np.zeros((self.amount, self.expected_nr_of_landmarks), dtype=int)
        self.scores = np.concatenate((self.scores, new_scores), axis=1)

        # Then landmark classes
        new_classes = np.zeros((self.amount, self.expected_nr_of_landmarks), dtype=int)
        self.landmark_classes = np.concatenate(
            (self.landmark_classes, new_classes), axis=1
        )

        self.supported_nr_of_landmarks += self.expected_nr_of_landmarks

    def set_map_view(self, map_id):
        """
        Sets the map view (what map should be used in the other methods?)

        Args:
          - map_id: the index of the map to set the view to
        """
        self.map_id = map_id

    def add_landmark(self, landmark, landmark_class=0, score=1):
        """
        Add a landmark to the map.

        Resets the corresponding kdtree, but does not (yet) build a new one

        Args:
          landmark: A 2x3 array representing the landmark
          landmark_class: The class of the landmark (or tag in other words)
          score: The likelyhood of the landmarks existence
        """

        map_id = self.map_id

        # Sometimes a map expansion is needed...
        if len(self) == self.supported_nr_of_landmarks:
            self.expand_map()

        # Add it to the map
        offset = len(self)
        self.landmarks[map_id, :, 3 * offset : (offset + 1) * 3] = landmark

        self.landmark_classes[map_id, offset] = landmark_class
        self.scores[map_id, offset] = score

        self.sizes[map_id] += 1

        # Reset kdtree
        self.kdtree[map_id] = None

    def remove_landmark(self, offset: float):
        """
        Removes a landmark from the map. This is actually quite a costly operation
        because it has to make sure the landmark array does not contain gaps

        Args:
            - offset: the landmark offset (or index) to remove from the current map
        """

        map_id = self.map_id

        self.landmarks[map_id, :, 3 * offset : 3 * (len(self) - 1)] = self.landmarks[
            map_id, :, 3 * (offset + 1) : 3 * len(self)
        ]

        self.scores[map_id, offset : len(self) - 1] = self.scores[
            map_id, offset + 1, len(self)
        ]
        self.landmark_classes[map_id, offset : len(self) - 1] = self.landmark_classes[
            map_id, offset + 1, len(self)
        ]
        self.sizes[map_id] -= 1

    """
    Simple getters and setters
    """

    def get_landmarks(self):
        return self.landmarks[self.map_id, :, : 3 * len(self)]

    def set_landmark_score(self, i, score):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        self.scores[self.map_id, i] = score

    def get_landmark_classes(self):
        return self.landmark_classes[self.map_id, : len(self)]

    def get_landmark_class(self, i):
        return self.landmark_classes[self.map_id, i]

    def get_landmark_score(self, i):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        return self.scores[self.map_id, i]

    def increment_landmark_score(self, i, amount=1):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        if self.scores[self.map_id][i] >= 5 and amount > 0:
            return
        self.scores[self.map_id][i] += amount

    def get_landmark(self, i):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        return self.landmarks[self.map_id][:, (3 * i) : (3 * i + 3)]

    def get_landmark_covariance(self, i):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        return self.landmarks[self.map_id][:, (3 * i + 1) : (3 * i + 3)]

    def get_landmark_mean(self, i):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        return self.landmarks[self.map_id][:, (3 * i)]

    def set_landmark_mean(self, i, mean):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        self.landmarks[self.map_id][:, (3 * i)] = mean

    def set_landmark_covariance(self, i, covariance):
        if i >= len(self):
            raise Exception(f"Index {i} out of range")
        self.landmarks[self.map_id][:, (3 * i + 1) : (3 * i + 3)] = covariance

    def __len__(self):
        """
        Returns the number of landmarks in this map
        """
        return int(self.sizes[self.map_id])

    def search_corresponding_landmark_index(self, candidate_landmarks_mu):
        """
        Searches the map for a correspoding landmark
        Returns the distance and index of closest landmark in this map

        It uses a cached tree and builds it first when needed

        Args:
          candidate_landmark_mu: a 1x2 vector representing the average position of the candidate landmark ([x y])
        """

        if len(self) == 0:
            return [], []

        if not self.kdtree[self.map_id]:
            landmark_mus = (self.landmarks[self.map_id][:, 0 : 3 * len(self) : 3]).T
            self.kdtree[self.map_id] = scipy.spatial.KDTree(landmark_mus)

        distance, index = self.kdtree[self.map_id].query(candidate_landmarks_mu)

        if len(candidate_landmarks_mu) == 2:
            index = [index]
            distance = [distance]

        return distance, index
