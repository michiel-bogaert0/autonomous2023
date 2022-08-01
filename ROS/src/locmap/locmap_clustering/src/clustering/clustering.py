from collections import deque

import numpy as np
from sklearn.cluster import DBSCAN


class Clustering:
    def __init__(
        self, expected_nr_of_samples=1000, nr_of_classes=2, eps=0.5, min_samples=5
    ):
        """
        Clustering constructor

        Args:
            - expected_nr_of_samples: the amount of samples you expect. This is just to prime the memory structures
            - nr_of_classes: the amount of "observation classes" you are going to have. Clustering gets scoped to a certain class
            - eps: a distance parameter that is used to determine to which cluster a particle is part of
            - min_samples: the minimal amount of samples required in a candidate cluster before it is deemed an actual cluster
        """

        self.expected_nr_of_samples = expected_nr_of_samples
        self.max_nr_of_samples = expected_nr_of_samples
        self.nr_of_classes = nr_of_classes
        self.eps = eps
        self.min_samples = min_samples

        self.sizes = np.zeros(nr_of_classes, dtype=int)
        self.samples = np.zeros((nr_of_classes, expected_nr_of_samples, 2))
        self.clusterers = [
            DBSCAN(eps=eps, min_samples=min_samples) for i in range(nr_of_classes)
        ]

        self.all_landmarks = []
        self.something_changed = True

    def expand_samples_array(self):
        """
        Extends the samples array in the case there are more samples than self.max_nr_of_samples
        """

        new_samples = np.empty((self.nr_of_classes, self.expected_nr_of_samples, 2))
        self.samples = np.concatenate((self.samples, new_samples), axis=1)

        self.max_nr_of_samples += self.expected_nr_of_samples

    def add_sample(self, sample, sample_class):
        """
        Adds a sample to the sample array

        Args:
            - sample: np.array of [x, y] position of the sample
            - sample_class: the sample class
        """

        if self.sizes[sample_class] >= self.max_nr_of_samples:
            self.expand_samples_array()

        self.samples[sample_class, self.sizes[sample_class]] = sample

        self.sizes[sample_class] += 1

        self.something_changed = True

    def cluster(self):
        """
        This function clusters the sample array, only when needed

        Returns:
            - A list of (X_i, 2) numpy arrays of landmark positions where X_i is the amount of landmarks in class i.
              The list has the same length as the amount of sample classes
        """

        if not self.something_changed:
            return self.all_landmarks

        self.all_landmarks = []

        self.clusterers = [
            DBSCAN(eps=self.eps, min_samples=self.min_samples)
            for i in range(self.nr_of_classes)
        ]

        for i, clusterer in enumerate(self.clusterers):

            samples = self.samples[i]

            clusterer.fit(samples)
            labels = clusterer.labels_

            nr_of_landmarks = len(set(labels)) - (1 if -1 in labels else 0)
            landmarks = np.zeros((nr_of_landmarks, 2))

            # Now get the centroids
            for j in range(nr_of_landmarks):
                landmarks[j] = np.mean(samples[labels == j], axis=0)

            self.all_landmarks.append(landmarks)

        return self.all_landmarks
