"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
"""

from math import pi

#Clase responsable de almacenar los datos necesarios para genere el label en KITTI format
class KittiLabel:
    def __init__(self, type=None, truncated=0.0, occluded=3, alpha=0.0, bbox=None, dimensions=None, location=None, rotation_y=0.0):
        self.type = type
        self.truncated = truncated
        self.occluded = occluded
        self.alpha = alpha
        self.bbox = bbox
        self.dimensions = dimensions
        self.location = location
        self.rotation_y = rotation_y
        self._valid_classes = ['Car', 'Van', 'Truck',
                               'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                               'Misc', 'DontCare']

    def set_type(self, obj_type: str):
        assert obj_type in self._valid_classes, "Campo type no valido, debe ser uno de: {}".format(
            self._valid_classes)
        self.type = obj_type

    def set_truncated(self, truncated: float):
        assert 0 <= truncated <= 1, """Campo truncated debe ser Float entre 0 y 1"""
        self.truncated = truncated

    def set_occluded(self, occluded: int):
        assert occluded in range(0, 4), """Campo Occluded debe ser Integer (0,1,2,3)"""
        self._occluded = occluded

    def set_alpha(self, alpha: float):
        assert -pi <= alpha <= pi, "Alpha debe estar entre [-pi..pi], alpha={}, rot_y={}, x={}, z={}".format(alpha,self.rotation_y,self.location[0],self.location[2])
        self.alpha = alpha

    def set_bbox(self, bbox):
        assert len(bbox) == 4, """ Campo Bbox debe ser el cuadro delimitador del objeto en la imagen 2D. 
                                Contiene las coordenadas de los píxeles de borde"""
        self.bbox = bbox

    def set_dimensions(self, dimensions):
        assert len(dimensions) == 3, """ Campo Dimensions debe ser alto,ancho,largo del objeto en metros """

        #Kitti expects height, width and length (z, y, x):
        self.dimensions = dimensions


    def set_location(self, obj_location):
        assert len(obj_location) == 3, """ Campo Location debe contener las coordenadas x,y,z del centro del objeto en coordenadas de camara"""
        self.location = obj_location

    def set_rotation_y(self, rotation_y: float):
        assert -pi <= rotation_y <= pi, "Rotation y debe estar en rando [-pi..pi]"
        self.rotation_y = rotation_y

    def get_label(self):
        """ generar el string correspondiente al label con toda la informacion necesaria"""
        str_truncated = "{:.2f}".format(self.truncated)
        str_occluded = "{}".format(self.occluded)
        str_alpha = "{:.2f}".format(self.alpha)
        str_bbox = "{0:.2f} {1:.2f} {2:.2f} {3:.2f}".format(self.bbox[0], self.bbox[1], self.bbox[2], self.bbox[3])
        str_dimensions = "{0:.2f} {1:.2f} {2:.2f}".format(self.dimensions[0], self.dimensions[1], self.dimensions[2])
        str_location = "{0:.2f} {1:.2f} {2:.2f}".format(self.location[0], self.location[1], self.location[2])
        str_rotation = "{:.2f}".format(self.rotation_y)

        return "{} {} {} {} {} {} {} {}".format(self.type, str_truncated, str_occluded, str_alpha, str_bbox, str_dimensions, str_location, str_rotation)

    def __str__(self):
        """ Returns the kitti formatted string of the datapoint if it is valid (all critical variables filled out), else it returns an error."""

        if self.bbox is None:
            bbox_format = " "
        else:
            bbox_format = " ".join([str(x) for x in self.bbox])

        return "{} {} {} {} {} {} {} {}".format(self.type, self.truncated, self.occluded, self.alpha, bbox_format, self.dimensions, self.location, self.rotation_y)
