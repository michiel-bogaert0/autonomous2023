import numpy as np
from sklearn.cluster import DBSCAN


class Clustering:
    def __init__(self, mode, expected_nr_of_samples=1000, eps=0.5, min_samples=5):
        """
        Clustering constructor

        Args:
            - mode: must be "global" or "local". In global mode, it keeps every single sample. In local mode it "forgets" samples
                    based on the expected_nr_of_samples parameter
            - expected_nr_of_samples: the amount of samples you expect. This is just to prime the memory structures
            - nr_of_classes: the amount of "observation classes" you are going to have. Clustering gets scoped to a certain class
            - eps: a distance parameter that is used to determine to which cluster a particle is part of
            - min_samples: the minimal amount of samples required in a candidate cluster before it is deemed an actual cluster
        """

        assert mode in [
            "global",
            "local",
        ], f"Mode should be 'global' or 'local'. Got: {mode}"

        self.mode = mode

        self.expected_nr_of_samples = expected_nr_of_samples
        self.max_nr_of_samples = expected_nr_of_samples
        self.eps = eps
        self.min_samples = min_samples

        self.size = 0
        self.samples = np.zeros((expected_nr_of_samples, 2))
        self.sample_classes = np.zeros(expected_nr_of_samples, dtype=int)
        self.clusterer = DBSCAN(eps=eps, min_samples=min_samples)

        self.all_landmarks = [0, np.array([]), np.array([])]
        self.something_changed = True

    def reset(self):
        self.size = 0
        self.samples = np.zeros((self.expected_nr_of_samples, 2))
        self.sample_classes = np.zeros(self.expected_nr_of_samples, dtype=int)
        self.clusterer = DBSCAN(eps=self.eps, min_samples=self.min_samples)

        self.all_landmarks = [0, np.array([]), np.array([])]
        self.something_changed = True

    def expand_samples_array(self):
        """
        Extends the samples array in the case there are more samples than self.max_nr_of_samples
        """

        new_samples = np.empty((self.expected_nr_of_samples, 2))
        self.samples = np.concatenate((self.samples, new_samples), axis=0)

        new_sample_classes = np.empty(self.expected_nr_of_samples, dtype=int)
        self.sample_classes = np.concatenate(
            (self.sample_classes, new_sample_classes), axis=0
        )

        self.max_nr_of_samples += self.expected_nr_of_samples

    def add_sample(self, sample, sample_class):
        """
        Adds a sample to the sample array

        Args:
            - sample: np.array of [x, y] position of the sample
            - sample_class: the sample class
        """

        if self.size >= self.max_nr_of_samples:
            if self.mode == "global":
                self.expand_samples_array()
            elif self.mode == "local":
                self.size = 0

        self.samples[self.size] = sample
        self.sample_classes[self.size] = sample_class

        self.size += 1

        self.something_changed = True

    def cluster(self):
        """
        This function clusters the sample array, only when needed

        Returns:
            [X, landmark_classes, landmarks] where landmark_classes is a nr x 1 array containing the landmark class
                and landmarks is an nr x 2 array containing the position (x, y)
        """

        if not self.something_changed:
            return self.all_landmarks

        self.all_landmarks = [0, np.array([]), np.array([])]

        self.clusterer = DBSCAN(eps=self.eps, min_samples=self.min_samples)

        # Fit the cluster to know where the landmarks are and fetch the actual samples
        # that are contributing to the landmarks
        self.clusterer.fit(self.samples)
        labels = self.clusterer.labels_

        nr_of_landmarks = len(set(labels)) - (1 if -1 in labels else 0)

        # Now get the centroids and determine the class
        landmarks = np.zeros((nr_of_landmarks, 2))
        classes = np.zeros(nr_of_landmarks, dtype=int)
        for j in range(nr_of_landmarks):
            landmarks[j] = np.mean(self.samples[labels == j], axis=0)

            values, counts = np.unique(
                self.sample_classes[labels == j], return_counts=True
            )
            classes[j] = values[np.argmax(counts)]

        self.all_landmarks = [nr_of_landmarks, classes, landmarks]

        return self.all_landmarks
