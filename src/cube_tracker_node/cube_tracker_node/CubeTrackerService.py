import numpy as np

CENTROIDDISTANCE_TO_SIDELENGTH_RATIO = 2.8
RADIAN_RATIO = 0.7
MARKER_TO_CUBE_RATIO = 4
EDGE_DISTANCE_TO_MARKERSIDELENGHT_RATIO = 10

DEBUG_MODE = False


# a Cube with 6 faces and each face having a marker on each corner can be tracked using this class.
# by default the size ration between cube and marker is expected to be 4, to track the cube a list of markers
# is expected in form of following dictionary
#       {'id': id,
#       'angle': angle,
#       'corners': [[x,y],[x,y],[x,y],[x,y]],
#       'centroid': (x,y)}
# with marker ids 0 - 23. the return of get_cube_spatial_info method has the same form for the cubeside.
# a cube side is expected to have markers with id % 4 = 0 for the marker in the upper left corner of the cube
# side and then clockwise to bottom left corner with id % 4 = 3 each having a distance of 1/10 of the marker sidelength
# between the marker and cube side.
# For example a cube with 8cm sidelengths is expected to have 4 markers with sidelength 2cm on each face.
# These markers are in each corner with a distance of 0.2cm to each side of the corner.
# If less than 4 markers are detected different substitution and approximation methods are used to robustely track the cube
class CubeDetector:
    def __init__(self, node=None):
        self.node = node

    def preselect_markers(self, marker_list):
        markers = sorted(marker_list, key=lambda found_markers: found_markers['id'])
        helper = [[], [], [], [], [], []]
        for marker in markers:
            idx = int(marker['id']/4)
            if idx < 6:
                helper[idx].append(marker)
        lenghts = [len(i) for i in helper]
        idx_side = lenghts.index(max(lenghts))
        return helper[idx_side]

    # this only works for markers with ids between 0 and 23
    def get_cube_spatial_info(self, markers):
        if len(markers) == 4:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info("4 markers found")
            return self.__get_geometrics(markers)

        elif len(markers) == 3:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info("3 markers found")
            # get the mean orthogonal length of all present markers
            orthogonal_lengths = self.__get_orth_lengths(markers)
            markers_with_substitution, cube_centroid = self.__three_marker_substitution(markers, orthogonal_lengths)
            return self.__get_geometrics(markers_with_substitution, cube_centroid)

        elif len(markers) == 2:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info("2 markers found")
            approx = []
            # get an approximation of the cubeside geoms for each marker
            # then return the mean of both approximated cubeside geoms
            for marker in markers:
                approx.append(self.__approx_cube_geoms([marker]))

            cube_geoms = {'id': approx[0]['id'],
                          'angle': (approx[0]['angle']+approx[1]['angle'])/2,
                          'corners': (approx[0]['corners']+approx[1]['corners'])/2,
                          'centroid': (approx[0]['centroid']+approx[1]['centroid'])/2}
            return cube_geoms

        elif len(markers) == 1:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info("1 markers found")
            return self.__approx_cube_geoms(markers)

        else:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info("Detection is only possible with one up to four markers")
            return None

    # uses one corner marker of a cubeside to approximate that sides geometrics
    def __approx_cube_geoms(self, markers):
        orth_lengths = self.__get_orth_lengths(markers)
        marker = markers[0]
        idx = marker['id'] % 4

        # depending on which corner (idx) the marker is use its centroid and corresponding scaled orthogonal length to approcimate cube centroid
        cube_centroid = np.array(marker['centroid']) - orth_lengths[idx]*CENTROIDDISTANCE_TO_SIDELENGTH_RATIO

        # use approximated cube centroid and scaled marker orthogonals to approximate cube corners
        cube_corners = cube_centroid + np.array([MARKER_TO_CUBE_RATIO*sl for sl in orth_lengths])

        cube_geoms = {'id': self.__get_side_number(marker['id']),
                      'angle': marker['angle'],
                      'corners': cube_corners,
                      'centroid': cube_centroid}
        return cube_geoms

    def __three_marker_substitution(self, markers, orth_lengths):
        ids = [m['id'] % 4 for m in markers]
        missing = set([i for i in range(0, 4)]) - set(ids)
        missing = list(missing)[0]
        if missing == 0:
            cube_centroid = np.array(markers[0]['centroid']) - (np.array(markers[0]['centroid']) - np.array(markers[2]['centroid']))/2
            substitute_id = markers[0]['id']-1
            substitute_centroid = cube_centroid + cube_centroid - np.array(markers[1]['centroid'])
        elif missing == 1:
            cube_centroid = np.array(markers[0]['centroid']) - (np.array(markers[0]['centroid']) - np.array(markers[1]['centroid']))/2
            substitute_id = markers[1]['id']-1
            substitute_centroid = cube_centroid + cube_centroid - np.array(markers[2]['centroid'])
        elif missing == 2:
            cube_centroid = np.array(markers[1]['centroid']) - (np.array(markers[1]['centroid']) - np.array(markers[2]['centroid']))/2
            substitute_id = markers[1]['id']+1
            substitute_centroid = cube_centroid + cube_centroid - np.array(markers[0]['centroid'])
        elif missing == 3:
            cube_centroid = np.array(markers[0]['centroid']) - (np.array(markers[0]['centroid']) - np.array(markers[2]['centroid']))/2
            substitute_id = markers[2]['id']+1
            substitute_centroid = cube_centroid + cube_centroid - np.array(markers[1]['centroid'])
        else:
            if self.node is not None:
                if self.node.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                    self.node.get_logger().info('wrong missing marker ID')
            return None

        substitute_corners = orth_lengths + substitute_centroid
        substitute_angle = sum([(m['angle']) for m in markers])/len(markers)

        substitute_marker = {'id': substitute_id,
                             'angle': substitute_angle,
                             'corners': substitute_corners,
                             'centroid': substitute_centroid}
        markers.insert(missing, substitute_marker)
        return markers, cube_centroid

    def __get_orth_lengths(self, markers):
        cubesides = [[], [], [], []]
        for m in markers:
            m['sides'] = np.array(m['corners']) - np.array(m['centroid'])
            for s in range(0,4):
                cubesides[s].append(m['sides'][s])

        return [sum(side)/len(side) for side in cubesides]

    def __get_geometrics(self, markers, cube_centroid=None):
        marker_centroids = [m['centroid'] for m in markers]
        if cube_centroid is None:
            cube_centroid = sum(np.array(marker_centroids)) / len(markers)

        marker_angles = [m['angle'] for m in markers]
        cube_angle = sum(marker_angles)/len(marker_angles)

        cube_geoms = {'id': self.__get_side_number(markers[0]['id']),
                      'angle': cube_angle,
                      'corners': self.__get_corners(marker_centroids, cube_centroid),
                      'centroid': cube_centroid}
        return cube_geoms

    def __get_side_number(self, idx):
        for i in range(0,6):
            if idx in range(i*4, i*4+4):
                return i
        return -1

    def __get_corners(self, marker_centroids, centroid_cube):
        corners = ((np.array(marker_centroids) - np.array(centroid_cube))/RADIAN_RATIO) + np.array(centroid_cube)
        return corners
