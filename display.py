class Display:
    """
    This is a library for doing a comprehensive, realtime display of all the states within the EKF.
    """

    def __init__(self):
        self.scalars = {}
        self.matrices = {}
        try:
            import curses
            self.curses_available = True
            self.screen = curses.initscr()
        except:
            print "Curses library not installed defaulting to standard console output"
            self.curses_available = False

    def __del__(self):
        if self.curses_available:
            import curses
            curses.endwin()

    def register_scalar(self, label, scalar):
        self.scalars[label] = scalar

    def register_scalars(self, scalars):
        for label, scalar in scalars.items():
            self.register_scalar(label, scalar)

    def register_matrix(self, label, matrix):
        self.matrices[label] = matrix

    def register_matrices(self, matrices):
        for label, matrix in matrices.items():
            self.register_matrix(label, matrix)

    def display_matrix(self, m, x, y, precision=2, title=None):
        a, b = self.get_matrix_display_size(m, precision=precision)
        c, d = self.screen.getmaxyx()
        if x + a < c and y + b < d: # if there's space to draw it, go for it
            rows, cols = m.shape
            if title:
                self.screen.addstr(x, y, title)
                x += 1
            self.screen.addstr(x, y, "[")
            self.screen.addstr(x, cols*(4+precision)+y+1, "]")
            self.screen.addstr(rows+x-1, y, "[")
            self.screen.addstr(rows+x-1, cols*(4+precision)+y+1, "]")
            for row in range(rows):
                for col in range(cols):
                    self.screen.addstr(row+x, col*(4+precision)+y+1, "%+.*f," % (precision, m[row, col]))
            return rows+x, cols*(4+precision)+y+2 # returns the position of the bottom right corner plus one for padding
        else: # not enough room to draw the matrix
            return x + 1, y + 1

    def get_matrix_display_size(self, m, precision=2, title=True):
        rows, cols = m.shape
        if title:
            rows += 1
        return rows, cols*(4+precision)+1 # returns the position of the bottom right corner plus one for padding

    def display_state(self, precision):
        self.screen.erase()
        rows, cols = self.screen.getmaxyx()
        scalar_count = len(self.scalars)
        self.screen.addstr(0, 0, "Shumai: the Extended Kalman Filter for aircraft")
        i = 1
        x, y = 2, 0
        for s in self.scalars.keys():
            if y + precision + len(s) > cols:
                x += 1
                y = 0
            if x < rows and y + precision + len(s) < cols:
                self.screen.addstr(x, y, "%s: %+0.*f" % (s, precision, self.scalars[s]))
            y += 20 + precision
            i += 1
        x, y = x + 2, 0
        maxheight = 0
        for m in self.matrices.keys():
            matrix = self.matrices[m]
            a, b = self.get_matrix_display_size(matrix, precision=precision)
            if a > maxheight:
                maxheight = a
            if b + y > cols-10:
                y = 0
                x += maxheight + 1
                maxheight = 0
            c, d = self.display_matrix(matrix, x, y, precision=precision, title=m)
            y += b + 3
        # point the cursor in the bottom right corner so it doesn't hide anything
        self.screen.move(rows-1, cols-1)

    def draw(self, precision=5):
        self.display_state(precision=precision)
        rows, cols = self.screen.getmaxyx()
        self.screen.move(rows-1, cols-1)
        self.screen.refresh()
        self.scalars = {}
        self.matrices = {}
