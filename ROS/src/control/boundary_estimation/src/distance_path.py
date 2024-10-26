class DistancePath:
    def __init__(self):
        """
        Initializes the DistancePath class, which computes a middle path
        between left and right boundary cones by calculating midpoint points.
        """
        self.current_left_cone = None
        self.current_right_cone = None

    def get_path(self, left_boundary, right_boundary):
        """
        Generates a path through the midpoints between corresponding cones on
        the left and right boundaries.

        Parameters:
        - left_boundary: List of cones on the left boundary, each with x, y, and id attributes.
        - right_boundary: List of cones on the right boundary, each with x, y, and id attributes.

        Returns:
        - List of midpoint points [(mid_x, mid_y, left_id, right_id), ...] between the left and right boundaries.
        """
        # Initialize boundary lists without the root
        self.left_boundary = left_boundary[1:]
        self.right_boundary = right_boundary[1:]
        self.left_boundary_copy = left_boundary[1:]
        self.right_boundary_copy = right_boundary[1:]
        self.midpoints_with_ids = []

        # Set the first cone for each boundary if available
        if self.left_boundary:
            self.current_left_cone = self.left_boundary.pop(0)
        if self.right_boundary:
            self.current_right_cone = self.right_boundary.pop(0)

        # Start the midpoint path by adding the initial midpoint if both cones exist
        if self.current_left_cone and self.current_right_cone:
            self.add_midpoint()

        # Traverse through boundaries until one is exhausted
        while self.left_boundary and self.right_boundary:
            # Compute distances to the next cone on each boundary
            dist_to_next_right = self.distance(
                self.right_boundary[0], self.current_left_cone
            )
            dist_to_next_left = self.distance(
                self.left_boundary[0], self.current_right_cone
            )

            # Move to the closest cone and add the corresponding midpoint
            if dist_to_next_right <= dist_to_next_left:
                self.current_right_cone = self.right_boundary.pop(0)
            else:
                self.current_left_cone = self.left_boundary.pop(0)
            self.add_midpoint()

        # Generate final midpoint list with actual (x, y) positions and IDs
        middle_path = [
            (
                (left_cone.x + right_cone.x) / 2,
                (left_cone.y + right_cone.y) / 2,
                left_cone.id,
                right_cone.id,
            )
            for left_id, right_id in self.midpoints_with_ids
            for left_cone in self.left_boundary_copy
            if left_cone.id == left_id
            for right_cone in self.right_boundary_copy
            if right_cone.id == right_id
        ]

        return middle_path

    def distance(self, cone1, cone2):
        """
        Calculates the squared Euclidean distance between two cones.
        """
        return (cone1.x - cone2.x) ** 2 + (cone1.y - cone2.y) ** 2

    def add_midpoint(self):
        """
        Adds the current midpoint between `self.current_left_cone` and `self.current_right_cone`
        to the midpoint list, storing only their IDs.
        """
        self.midpoints_with_ids.append(
            (self.current_left_cone.id, self.current_right_cone.id)
        )
