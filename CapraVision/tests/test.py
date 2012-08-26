import os
os.chdir(os.path.join(os.getcwd(), ".."))
os.sys.path.insert(0,os.getcwd()) 

from gi.repository import Gtk
from gi.repository import GObject
GObject.threads_init()

import cv2

import gui, sources, chain

def source_image():
    image = cv2.imread('0-157.png')
    return image

def test_viewer():
    #s = sources.Webcam()
    c = chain.FilterChain()
    c.add_filter(noop)
    w = gui.WinViewer(c, noop)
    w.window.show_all()
    Gtk.main()
    
def testFileChooser():
    dialog = Gtk.FileChooserDialog("Please choose a filterchain file", None,
                                   Gtk.FileChooserAction.OPEN,
                                   (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                    Gtk.STOCK_OK, Gtk.ResponseType.OK))
    ff = Gtk.FileFilter()
    ff.set_name('Filterchain')
    ff.add_pattern('*.filterchain')
    
    dialog.set_filter(ff)
    response = dialog.run()
    if response == Gtk.ResponseType.OK:
        print "Select clicked"
        print "Folder selected: " + dialog.get_filename()
    elif response == Gtk.ResponseType.CANCEL:
        print "Cancel clicked"
    else:
        print 'Aucun'
        dialog.destroy()

def testConfig():
    import ConfigParser
    

if __name__ == '__main__':
    testFileChooser()
