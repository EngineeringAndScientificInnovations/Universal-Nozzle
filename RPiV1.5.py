# -*- coding: utf-8 -*-
"""
Created on Tue Oct 07 23:18:45 2014
@author: Nick
This is not terribly well-written in retrospect. The assumption is made at many
points that variables are in the global namespace. This is okay, but not great.
For instance, the pi for mixing the powder, powderMixer is never explicitly
passed to the function which fires the pins, but is defined before that
function is ever called, so it works out. Not good practice, but it's working.
Fix if you feel the need by either explicitly placing variables like that in the
global namespace with the 'global' keyword (see how ImQueue is handled for instance),
or finding a way to explicitly pass the needed variables into the local function
space (hard or impossible with certain callback functions).
"""
from threading import Thread, Lock
from collections import deque
import picamera, picamera.array
import time, bisect
import RPi.GPIO as GPIO
from datetime import datetime
from numpy import nan, isnan, abs, average, zeros
from cv2 import GaussianBlur, moments, imwrite
   

def suppressFire_callback(channel,flag=0):
    """This function is called when the UVTron sends the trigger pulse on
    the GPIO pin. It is set up as a callback function, so only gets
    executed when the pin fires.
    """
    if not flag: # debugging stuff
        print 'UVTron just told me about a fire!', GPIO.input(channel)
    x,y = nan, nan    # initialize fire location to NaN
    while isnan(x) or isnan(y):    # no fire ball found yet
#       Need to do some sort of check to filter out random spikes here
        ti = time.time()
        while GPIO.input(channel): # still have high signal from UVTron, so keep checking for fire
#            print 'signal went high for some reason'
            if time.time() - ti >= 0.01: # haven't seen anything. question validity
                print "Something may be wrong with the UVTron signal"
                return
#        FireImage = abs(average(ImQueue[-1],-1) - average(ImQueue[0],-1)) # an attempt to average two consecutive frames. Didn't work very well
        FireImage = average(ImQueue[0],-1)    # grayscale the image
        x,y = findFire(FireImage)            # locate the fire centroid
#    fo = '-'.join(map(str, datetime.now().timetuple()[:6])) # timestamp the image (will not be accurate unless there is internet connection)
#    imwrite('fire'+fo+'.bmp',FireImage)    # write image to file. usefule for debugging
    xdivtmp, ydivtmp = xdivs[:], ydivs[:] # divide the frame into sections
    bisect.insort(xdivtmp,x)   # Insert the fire coordinates into the protection grid
    bisect.insort(ydivtmp,y)
    xzone = xdivtmp.index(x) - 1   # Find the grid coordinates
    yzone = ydivtmp.index(y) - 1
    del xdivtmp, ydivtmp
    firePorts((xzone,yzone))    # fire the corresponding ports for grid location
    print 'Fire at (%.2f,%.2f) in zone %d,%d\nFiring ports %d & %d' % ((x,y,xzone,yzone,) + fireDict[(xzone,yzone)])

def findFire(data):
    '''Locates the brightest area in the frame by applying a differential
        gaussian filter and creating a byte array of light vs dark. The centroid
        of the light region is found and returned as the location of the fire'''
    data = GaussianBlur(data,(3,3),2)    # assumes grayscale input
    mask = zeros(data.shape)            # blank mask
    thresh = 40                            # would like this to be more intelligent...
#    thresh = (data.mean() + data.max())/2    #... but this fails for various reasons
#    print thresh                            #...
#    mask[data > (data.mean() + data.max())/2] = 1 #...
    mask[data > thresh] = 1                # fill in mask with everything brighter than threshold
    mom = moments(mask)                    # get moments of mask (OpenCV call)
#    imwrite('mask{0}{1}{2}.bmp'.format(mom['m00'],mom['m02'],mom['m20']),mask)
    if mom['m00']:    # at least some part of the image exceeded threshold
        # centroid formula here: http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#moments
        x, y = mom['m10']/mom['m00'], mom['m01']/mom['m00'] # calculate the moment
    else:            # no part of image above threshold
#        print 'nope'
        x, y = nan, nan
    return x, y

def pictureQueue(res,bright,con,fps,gains):
    '''Keeps a running queue of three pictures for comparison and fire location.
    Uses a deque data structure for efficient storage (popleft and maxlen are handy).
    This currently uses the picamera API, but would be far more efficient if
    converted over to OpenCV.VideoCapture API, which now supports the PiCam.
    See here: http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture
    '''
    global ImQueue, Flag
    ImQueue = deque(maxlen=3)
#    window = (1400,150,300,180)
    with picamera.PiCamera() as cam:
#        cam.preview_fullscreen = False
#        cam.preview_window = window
        # all of these values have been tweaked extensively. May need further adjusting.
        # OpenCV has methods to set these things as well. See here: http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-set
        cam.iso = 800
        cam.awb_mode = 'off'
        cam.awb_gains = gains
        cam.resolution = res
        cam.shutter_speed = 100
        cam.brightness = bright
        cam.contrast = con
        cam.framerate = fps
        cam.led = False
#        cam.start_preview()
        cam.rotation = 180
        time.sleep(1)
        print 'camera is live'
        Flag = 1
        with picamera.array.PiRGBArray(cam) as output: # the following represents 80 hours investment... There is no faster way that I'm aware of to get images from the PiCam using this API
            count = 0
            t1 = time.time()
            for im in cam.capture_continuous(output,'rgb',use_video_port = True):
                count += 1
                ImQueue.append(output.array)
                output.truncate(0)
                if not count%1000:
                    print '%d fps' % int(1000/(time.time()-t1))
                    count, t1 = 0, time.time()
                if not Flag:
                    cam.stop_preview()
                    break
    print 'camera closed'

def firePorts_mosfet(dat):
    '''Activates the correct GPIO pins for the in-house mosfet board. This
    function is used for the 7-port nozzle with the extra port used for powder
    mixing.'''
    firePins = [powderMixer]    # pin to fire the powder mixer add to pin list
    for pin in fireDict[dat]:
        firePins.append(solPins[pin])   # expand list of pins which need firing
    GPIO.output(firePins,1)             # fire solenoids
    GPIO.output(sigPin,1)               # light LED
    time.sleep(2)                       # fire for 2 seconds
    GPIO.output(firePins,0)             # turn off soleniods
    GPIO.output(sigPin,0)               # turn off LED
    if not GPIO.input(gatePin):         # if UVTron is still signalling
        suppressFire_callback(gatePin,1)    # start over and callback again

def firePorts_mosfet_8port(dat):
    '''Activates the correct GPIO pins for the in-house mosfet board. 8-port
    nozzle with no powder mixing.'''
    for pin in fireDict[dat]:
        firePins.append(solPins[pin]) 
    GPIO.output(firePins,1)
    GPIO.output(sigPin,1)
    time.sleep(2)
    GPIO.output(firePins,0)
    GPIO.output(sigPin,0)
    if not GPIO.input(gatePin):
        suppressFire_callback(gatePin,1)

def fireAllPorts_callback(channel):
    '''Activates GPIO pins to fire all ports in case of override. Should be linked
    to override pin as a callback function. Assumes there is apowder mixer installed
    '''
    GPIO.output(solPins + (powderMixer,),1)
    time.sleep(3)
    GPIO.output(solPins + (powderMixer,),0)

def calibrate():
    """Camera calibration loop. Legacy code that has been passed down from
    generation to generation whose meaning has been obscured by the sand of time.
    But seriously, though, no idea what most of this is supposed to be doing.
    """
    rgbavg = 100
    counter = 1
    tog1 = 2
    tog2 = 2
    
    with picamera.PiCamera() as cam:
        cam.resolution = (100,60)
        cam.start_preview()
        time.sleep(1)
        cam.iso = 800
        cam.shutter_speed = 100
        cam.framerate = 30
        gains = cam.awb_gains
        cam.awb_mode = "off"
        cam.awb_gains = gains
        bright = 70
        con = 70
        cam.contrast = con
        cam.brightness = bright
        bvar = 0
        cvar = 0
        gmax = 100
        scord = 3
        mcord = 3
        with picamera.array.PiRGBArray(cam) as output:
            while rgbavg > 12 or gmax > 150:
                if not tog1%2:
                    if not tog2%2:
                        bvar += 5
                        cam.brightness = bright + bvar
                    else:
                        bvar += 5
                        cam.brightness = bright - bvar
                else:
                    if not tog2%2:
                        cvar += 5
                        cam.contrast = con + cvar
                        tog2 += 1
                    else:
                        cvar += 5
                        cam.contrast = con - cvar
                        tog2 += 1
                tog1 += 1
                print counter
                counter += 1
                if counter > 6:
                    cam.brightness = 20
                    cam.contrast = 70
                    break
                brightP = cam.brightness
                conP = cam.contrast
                cam.capture(output,'rgb',use_video_port=True)
                img = output.array
                output.truncate(0)
                rtot, gtot, btot, gmax = 0, 0, 0, 0
                ysize,xsize = cam.resolution
                for s in range(xsize/2):
                    s *= 2
                    for m in range(ysize/2):
                        m *= 2
                        r,g,b = img[s, m]
                        rtot += r
                        gtot += g
                        btot += b
                        if g > gmax:
                            gmax = g
                            scord = s
                            mcord = m
        
                ravg = rtot/(xsize*ysize/4)/2.55
                gavg = gtot/(xsize*ysize/4)/2.55
                bavg = btot/(xsize*ysize/4)/2.55
                rgbavg = (ravg + gavg + bavg)/3
        
                if (0 < scord < xsize) and (0 < mcord < ysize):
                    r, g1, b = img[scord,mcord]
                    r, g2, b = img[scord+1, mcord+1]
                    r, g3, b = img[scord, mcord+1]
                    r, g4, b = img[scord-1, mcord]
                    r, g5, b = img[scord, mcord-1]
                    gmaxarea = (g1 + g2 + g3 + g4 + g5)/5.
                else:
                    gmaxarea = gmax
                    print 'safeguard active'
        
                print rgbavg, 'percent white'
                print gmaxarea, 'g max area'
        cam.stop_preview()
        cam.start_preview()    
        time.sleep(2)
        cam.stop_preview()
        return gains, bright, con
        


# Set up serial connection, gpio pins
#---
gatePin = 24        # pin for logic gate. usually just the signal from UVTron
sigPin = 5          # signal pin to LED for visual reference
powderMixer = 6     # solenoid pin to fire the powder-mixing port
overridePin = 23    # pin attached to pilot override switch (if implemented)
solPins = (12, 19, 26, 21, 20, 16, 13, 6)   # pins attached to solenoids (mosfets)
sol0, sol1, sol2, sol3, sol4, sol5, sol6 = solPins
GPIO.setmode(GPIO.BCM)  # use Broadcom's numbering system

GPIO.setup(gatePin,GPIO.IN,pull_up_down = GPIO.PUD_OFF)         # UVTron as input
GPIO.setup(overridePin,GPIO.IN,pull_up_down = GPIO.PUD_DOWN)    # override switch as input
GPIO.setup(sigPin,GPIO.OUT)             # LED as output
GPIO.setup(powderMixer,GPIO.OUT)        # powder mixer as output
GPIO.setup(solPins,GPIO.OUT)            # solenoids as output
for pin in solPins:
    # turn all solenoids off. their state is nondeterministic when requested from
    # the operating system, so we need to ensure they are all off, by forcing them.
    # this is why the solenoids may be active for a short moment on system startup.
    # this is also why the manual switch is necessary on system startup.
    GPIO.output(pin,0)

GPIO.output(sigPin,1)   # turn on LED to inform of system status
#---

# Map the ports to the frame grid. These may need to be derived by trial/error.
# This mapping will need to be adjusted for specific environments.
#---
#portcombos = [(4,5), (4,5), (3,4), (3,4), (3,4), (2,4), (2,4), (1,3), (1,2)] #Dry Bay mapping
portcombos = [(5, 7), (4, 7), (6, 7), (3, 7), (2, 7),
              (5, 6), (4, 6), (1, 7), (3, 6), (2, 6),
              (4, 5), (3, 5), (1, 6), (2, 4), (2, 3),
              (0, 5), (0, 4), (0, 6), (0, 3), (0, 2),
              (0, 5), (0, 4), (0, 1), (0, 3), (0, 2)] # Burn box test mapping

#portcombos = portcombos[::-1]
res = 50,30             # camera resolution
xgrid, ygrid = 5, 5     # protection grid setup
grid = [(x,y) for y in range(ygrid) for x in range(xgrid)] # (xgrid) x (ygrid) grid locations
fireDict = dict(zip(grid,portcombos)) # Hash grid locations to ports

xsize , ysize = res # this block of code constructs the protection grid divisions
xsplit = xsize / xgrid
ysplit = ysize / ygrid
xdivs = range(0, xsize + xsplit, xsplit)
ydivs = range(0, ysize + ysplit, ysplit)

#---


# Camera calibration
#---
fps = 90                # requested camera framerate. 90fps is not acheivable in actuality, but let's try anyway.
gains, bright, con = calibrate()    # run mystery calibration loop
#---

# choose which function to setup as port-firing callback
firePorts = firePorts_mosfet_8port
#firePorts = firePorts_mosfet
if __name__ == '__main__':
    try:
        lock = Lock()
        t1 = Thread(target = pictureQueue, args = (res,bright,con,fps,gains,))
        t1.daemon = True
        t1.start()
        GPIO.add_event_detect(gatePin, GPIO.FALLING, callback = suppressFire_callback, bouncetime = 300)
#        GPIO.add_event_detect(overridePin, GPIO.FALLING, callback = fireAllPorts_callback,bouncetime = 500)
        time.sleep(1)
        while not ('ImQueue' in globals() and len(ImQueue) > 2):
            time.sleep(0.5)
        GPIO.output(sigPin,0)       # turn off LED to signal system ready state
        while True:
            time.sleep(1)           # let the threads do their thing
    except KeyboardInterrupt:
        print '\nkilling program...'
        GPIO.cleanup()
        Flag = 0
        t1.join()
