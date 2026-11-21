"""Capture only the Gazebo XWayland window for local visual verification."""
import ctypes as C,ctypes.util,re,subprocess
from pathlib import Path
from PIL import Image
text=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
m=re.search(r'(0x[0-9a-f]+) "Gazebo Sim": \("gz-sim-gui"',text)
if not m:raise SystemExit('Gazebo window not found')
X=C.CDLL(ctypes.util.find_library('X11'));X.XOpenDisplay.restype=C.c_void_p;d=X.XOpenDisplay(None);w=int(m[1],16)
X.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint)]
root=C.c_ulong();x=C.c_int();y=C.c_int();width=C.c_uint();height=C.c_uint();border=C.c_uint();depth=C.c_uint()
X.XGetGeometry(d,w,C.byref(root),C.byref(x),C.byref(y),C.byref(width),C.byref(height),C.byref(border),C.byref(depth))
class XI(C.Structure):
    _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int),('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]
X.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int];X.XGetImage.restype=C.POINTER(XI)
im=X.XGetImage(d,w,0,0,width.value,height.value,0xffffffff,2)
if not im:raise SystemExit('No window image')
i=im.contents
if i.bits_per_pixel!=32:raise SystemExit(f'Unsupported pixel format {i.bits_per_pixel}')
raw=C.string_at(i.data,i.height*i.bytes_per_line)
p=Path(__file__).resolve().parents[1]/'assets/gazebo_window.png';Image.frombytes('RGB',(i.width,i.height),raw,'raw','BGRX',i.bytes_per_line).save(p)
X.XDestroyImage.argtypes=[C.POINTER(XI)];X.XDestroyImage(im);X.XCloseDisplay.argtypes=[C.c_void_p];X.XCloseDisplay(d);print(p)
