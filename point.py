# ------------------------------------------------------------------------
#  point class
# ------------------------------------------------------------------------
class point:
    """A point identified by (x,y) coordinates"""
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    
    def __add__(self, p):
        return point(self.x+p.x, self.y+p.y)
    
    def __sub__(self, p):
        return point(self.x-p.x, self.y-p.y)
    
    def __mul__(self, scalar):
        return point(self.x*scalar, self.y*scalar)
    
    def __div__(self, scalar):                        ## python 2.7
        return point(self.x/scalar, self.y/scalar)
    
    def __truediv__(self, scalar):                    ## python 3.5
        return point(self.x/scalar, self.y/scalar)
    
    def __str__(self):
        return "(%s, %s)" % (self.x, self.y)
    
    def __repr__(self):
        return "%s(%r, %r)" % (self.__class__.__name__, self.x, self.y)
    
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def distanceTo(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()
    
    def asTuple(self):
        return (self.x, self.y)
    
    def clone(self):
        return point(self.x, self.y)
    
    def moveTo(self, x, y):
        self.x = x
        self.y = y
    
    def translate(self, dx, dy):
        self.x = self.x + dx
        self.y = self.y + dy

    def toInteger(self):
        self.x = int(self.x + 0.5)
        self.y = int(self.y + 0.5)
    
    def toFloat(self):
        self.x = float(self.x)
        self.y = float(self.y)


