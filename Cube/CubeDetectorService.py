import numpy as np



CENTROIDDISTANCE_TO_SIDELENGTH_RATIO = 2.8
RADIAN_RATIO = 0.7
MARKER_TO_CUBE_RATIO = 4
EDGE_DISTANCE_TO_MARKERSIDELENGHT_RATIO = 10

class CubeDetector:
    some_parameter = None

    def __init__(self):

        print("cube detector service initialized")

    def substitute_markers(self, markers):
        l = self.get_orth_lengths(markers)
        if len(markers) == 4:
            #print("4 markers found")
            return None, self.get_cubeside_geometrics(markers)
            #return(markers)
        elif len(markers) == 3:
            #print("3 markers found")
            markers_w_subst, cen = self.three_marker_substitution(markers, l)
            return cen, self.get_cubeside_geometrics(markers_w_subst)
        elif len(markers) == 2:
            #print("2 markers found")
            approx = []
            for marker in markers:
                approx.append(self.approx_cube_geoms([marker])[1])
            cube_side = {'sidenumber': approx[0]['sidenumber'],
                         'centroid': (approx[0]['centroid']+approx[1]['centroid'])/2,
                         # 'radius': self.__get_radius(markercentroids, centroid),
                         'corners': (approx[0]['corners']+approx[1]['corners'])/2,
                         'angle': (approx[0]['angle']+approx[1]['angle'])/2}
            return cube_side['centroid'], cube_side
        elif len(markers) == 1:
            #print("1 markers found")
            return self.approx_cube_geoms(markers)

    def approx_cube_geoms(self, markers):
        l = self.get_orth_lengths(markers)
        marker = markers[0]

        for i in range(0, 4):
            if marker['id'] % 4 == i:
                cube_centroid = np.array(marker['centroid']) - l[i]*CENTROIDDISTANCE_TO_SIDELENGTH_RATIO
                cube_corners = cube_centroid + np.array([MARKER_TO_CUBE_RATIO*sl for sl in l])
        cube_side = {'sidenumber': self.__get_cubeside_number(marker['id']),
                     'centroid': cube_centroid,
                     # 'radius': self.__get_radius(markercentroids, centroid),
                     'corners': cube_corners,
                     'angle': self.__get_cubeside_angle([m['angle'] for m in markers])}

        return cube_centroid, cube_side

    def three_marker_substitution(self, markers, orth_lengths):
        ids = [m['id']%4 for m in markers]
        #print(ids)
        missing = None
        for i in range(0, 4):
            if i not in ids:
                missing = i
        #print(missing)
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
            print('wrong missing marker ID')

        substitute_corners = orth_lengths + substitute_centroid
        substitute_angle = sum([(m['angle']) for m in markers])/len(markers)

        substitute_marker = {'id': substitute_id,
                             'angle': substitute_angle,
                             'corners': substitute_corners,
                             'centroid': substitute_centroid}
        markers.insert(missing, substitute_marker)
        return markers, cube_centroid


    def get_orth_lengths(self, markers):
        cubesides = [[], [], [], []]
        for m in markers:
            m['sides'] = np.array(m['corners']) - np.array(m['centroid'])
            for s in range(0,4):
                cubesides[s].append(m['sides'][s])

        return [sum(side)/len(side) for side in cubesides]


    def get_cubeside_geometrics(self, markers):

        marker_centroids = [m['centroid'] for m in markers]
        centroid = sum(np.array(marker_centroids)) / len(markers)
        #print(marker_centroids)
        #print(centroid)
        cube_side = {'sidenumber': self.__get_cubeside_number(markers[0]['id']),
                    'centroid': centroid,
                    #'radius': self.__get_radius(markercentroids, centroid),
                    'corners': self.__get_cubecorners(marker_centroids, centroid),
                    'angle': self.__get_cubeside_angle([m['angle'] for m in markers])}

        return cube_side

    def __get_cubeside_number(self, idx):
        for i in range(0,6):
            if idx in range(i*4, i*4+4):
                return i
        return -1

    def __get_radius(self, marker_centroids, centroid_cube):
        radiae = np.array(marker_centroids) - np.array(centroid_cube)
        r = np.sqrt((sum([x[0] ** 2 for x in radiae]) / 4) + (sum([y[1] ** 2 for y in radiae]) / 4)) / 0.7
        return r

    def __get_cubeside_angle(self, angles):
        return sum(angles)/len(angles)

    def __get_cubecorners(self, marker_centroids, centroid_cube):
        corners = ((np.array(marker_centroids) - np.array(centroid_cube))/RADIAN_RATIO) + np.array(centroid_cube)
        return corners
