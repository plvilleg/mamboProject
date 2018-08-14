""" 

Demonstrates how to stream using the eStream functions.

"""

from labjack import ljm
import time as tm
import sys
import numpy as np
from datetime import datetime
#from oct2py import octave
import matplotlib.pyplot as plt
from scipy.signal import find_peaks_cwt
import scipy.fftpack as ft
import peakutils
#from peakdetect import peakdetect
handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctETHERNET, "192.168.254.100")
OFFSET= 0.131
MULTIPLICITY=1400
ANS=[0, 0, 0, 0]
DIGITAL = ["FIO1", "FIO4", "FIO2", "FIO0"]
def seleccionador (x):
    ljm.eWriteName(handle, DIGITAL[x], 1)
   
def silenciador (x, peak):

    # print ("hola")
    for dato in peak:
	# print (dato)
	if(dato != 0):
	    # end = datetime.now()
	    ljm.eWriteName(handle, DIGITAL[x], 0)   
	    return (0)

    return (1)


def generador_Frecuecia (f3, f4, nombre):

    fs = 50000
    # f = 500
    #ciclo1=f1/400.0000000
    #ciclo1=np.ceil(ciclo1)
    #ciclo2 = f2 / 400.0000000
    #ciclo2 = np.ceil(ciclo2)
    #duration1 = np.float32(ciclo1/f1)
    #duration2 = np.float32(ciclo2 / f2)
    #samples1 = (np.sin(2 * np.pi * np.arange(fs * duration1) * f1 / fs)).astype(np.float32)
    #samples1 = 2.0 * samples1 + 2.5
    #samples2 = (np.sin(2 * np.pi * np.arange(fs * duration2) * f2 / fs)).astype(np.float32)
    #samples2 = 2.0 * samples2 + 2.5
    #plt.plot(samples)
    #plt.show()

    # print(len(samples1))
    # MAX_REQUESTS = 1000  # The number of eStreamRead calls that will be performed.

    # Open first found LabJack

    # handle = ljm.openS("ANY", "ANY", "ANY")

    info = ljm.getHandleInfo(handle)
    print("Opened a LabJack with Device type: %i, Connection type: %i,\n" \
          "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" % \
          (info[0], info[1], info[2], ljm.numberToIP(info[3]), info[4], info[5]))


    # Desactivacion de parlantes
    ljm.eWriteName(handle, "FIO1", 0)
    ljm.eWriteName(handle, "FIO4", 0)
    ljm.eWriteName(handle, "FIO2", 0)
    ljm.eWriteName(handle, "FIO0", 0)

    # Setup Stream Out
    #OUT_NAMES = ["DAC0", "DAC1"]
    #NUM_OUT_CHANNELS = len(OUT_NAMES)
    #outAddress1 = ljm.nameToAddress(OUT_NAMES[0])[0]
    #outAddress2 = ljm.nameToAddress(OUT_NAMES[0])[1]
    # Allocate memory for the stream-out buffer
    #ljm.eWriteName(handle, "STREAM_OUT0_TARGET", 1000)
    #ljm.eWriteName(handle, "STREAM_OUT0_BUFFER_SIZE", 2048)
    #ljm.eWriteName(handle, "STREAM_OUT0_ENABLE", 1)
    #ljm.eWriteName(handle, "STREAM_OUT0_LOOP_SIZE", len(samples1))
    #ljm.eWriteName(handle, "STREAM_OUT1_TARGET", 1002)
    #ljm.eWriteName(handle, "STREAM_OUT1_BUFFER_SIZE", 2048)
    #ljm.eWriteName(handle, "STREAM_OUT1_ENABLE", 1)
    #ljm.eWriteName(handle, "STREAM_OUT1_LOOP_SIZE", len(samples2))
    
    freq1=80000000.000000000000000000/32
    freq1=freq1/f3
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_ENABLE", 0)
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_DIVISOR", 32)
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_ROLL_VALUE", freq1)
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_ENABLE", 1)
    ljm.eWriteName(handle, "DIO3_EF_ENABLE", 0)
    ljm.eWriteName(handle, "DIO3_EF_INDEX", 0)
    ljm.eWriteName(handle, "DIO3_EF_OPTIONS", 1)
    ljm.eWriteName(handle, "DIO3_EF_CONFIG_A", freq1/2)
    ljm.eWriteName(handle, "DIO3_EF_ENABLE", 1)
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_DIVISOR", 32)
    ljm.eWriteName(handle, "DIO_EF_CLOCK1_ROLL_VALUE", freq1)
    ljm.eWriteName(handle, "DIO3_EF_CONFIG_A", freq1/2)

    freq2=80000000.000000000000000000/32
    freq2=freq2/f4
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_ENABLE", 0)
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_DIVISOR", 32)
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_ROLL_VALUE", freq2)
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_ENABLE", 1)
    ljm.eWriteName(handle, "DIO5_EF_ENABLE", 0)
    ljm.eWriteName(handle, "DIO5_EF_INDEX", 0)
    ljm.eWriteName(handle, "DIO5_EF_OPTIONS", 2)
    ljm.eWriteName(handle, "DIO5_EF_CONFIG_A", freq2/2)
    ljm.eWriteName(handle, "DIO5_EF_ENABLE", 1)
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_DIVISOR", 32)
    ljm.eWriteName(handle, "DIO_EF_CLOCK2_ROLL_VALUE", freq2)
    ljm.eWriteName(handle, "DIO5_EF_CONFIG_A", freq2/2)




    #for i in range(0, len(samples1)):
    #   ljm.eWriteName(handle, "STREAM_OUT0_BUFFER_F32", samples1[i])
    #for i in range(0, len(samples2)):
    #    ljm.eWriteName(handle, "STREAM_OUT1_BUFFER_F32", samples2[i])

    tm.sleep(1)
    #ljm.eWriteName(handle, "STREAM_OUT0_SET_LOOP", 1)
    #ljm.eWriteName(handle, "STREAM_OUT1_SET_LOOP", 1)
    #print("STREAM_OUT0_BUFFER_STATUS = %f" % (ljm.eReadName(handle, "STREAM_OUT0_BUFFER_STATUS")))
    #print("STREAM_OUT0_BUFFER_STATUS = %f" % (ljm.eReadName(handle, "STREAM_OUT1_BUFFER_STATUS")))
    # Stream Configuration
    aScanListNames = ["AIN2"]  # Scan list names to stream
    numAddresses = len(aScanListNames)
    aScanList = ljm.namesToAddresses(numAddresses, aScanListNames)[0]
    scanRate = fs
    scansPerRead = int(scanRate / 2000)

    # Datos de Transformada de FFT
    T = 1.00000/fs
    x = np.linspace(0.00, scansPerRead*T, scansPerRead)
    xf = np.linspace(0.00, 1.00/(2.00*T), scansPerRead/2)
    # fig, (ax, bx) = plt.subplots(2, 1)

    # plt.ion()
    # Add the scan list outputs to the end of the scan list.
    # STREAM_OUT0 = 4800, STREAM_OUT1 = 4801, etc.
    # aScanList.extend([4800])  # STREAM_OUT0
    # If we had more STREAM_OUTs
    # aScanList.extend([4801]) # STREAM_OUT1
    # aScanList.extend([4802]) # STREAM_OUT2
    # aScanList.extend([4803]) # STREAM_OUT3

    try:
        # Configure the analog inputs' negative channel, range, settling time and
        # resolution.
        # Note when streaming, negative channels and ranges can be configured for
        # individual analog inputs, but the stream has only one settling time and
        # resolution.
        aNames = ["AIN2_NEGATIVE_CH", "AIN_ALL_RANGE", "STREAM_SETTLING_US",
                  "STREAM_RESOLUTION_INDEX"]
        aValues = [3, 10.0, 0, 0]  # single-ended, +/-10V, 0 (default),
        # 0 (default)
        ljm.eWriteNames(handle, len(aNames), aNames, aValues)

        i = 1
	j = 0
	print("Empieza")
        scanRate = ljm.eStreamStart(handle, scansPerRead, 1, aScanList, scanRate)
	while j<1:
	    i = 1
	    k = 0
	    h = 0
	    print(j)
	    while(k < 1000):		
    	    	k=k+1
		ret = ljm.eStreamRead(handle)
	    # str=tm.time()
	    seleccionador (j)
	    # end=tm.time()
	    while i:  # i <= M9A;X_REQUESTS:
		# for g in range(0,2):	        
		ret = ljm.eStreamRead(handle)
		data = ret[0][0:scansPerRead]
	        # print("Hola")
		# start2 = datetime.now()
	        yf = ft.fft(data)
	        yf= 2.0 / scansPerRead * np.abs(yf[:scansPerRead // 2])
		# print("Hola2")
		    #(peaks, indexes) = octave.findpeaks(yf, 'DoubleSided', 'MinPeakHeight', 0.04, 'MinPeakDistance', 100, 'MinPeakWidth', 0)
		    # indexes = find_peaks_cwt(yf, np.arange(1, 2))
	        indexes = peakutils.indexes(yf, thres=0.01 / max(yf), min_dist=100)
		print (indexes)
	        i = silenciador (j, indexes)
		h = h + 1
		# end2 = datetime()

		# end = datetime.now
		# plt.close()
		# print("\nTotal scans = %i" % (totScans))
	    # tt = (end - start).seconds + float((end - start).microseconds) / 1000000
	    tt = h * 0.0002		
	    # tt2= end-str
	    print("Tiempo 1000Hz = %f seconds" % (tt))
	    # print("Tiempo 500Hz = %f seconds" % (tt2))
	    ANS[j]=(tt-OFFSET)*MULTIPLICITY
	    j=j+1
        # print("LJM Scan Rate = %f scans/second" % (scanRate))
        # print("Timed Scan Rate = %f scans/second" % (totScans / tt))
        # print("Timed Sample Rate = %f samples/second" % (totScans * numAddresses / tt))
        # print("Skipped scans = %0.0f" % (totSkip / numAddresses))
    except ljm.LJMError:
        ljme = sys.exc_info()[1]
        print(ljme)
    except Exception:
        e = sys.exc_info()[1]
        print(e)
    ljm.eWriteName(handle, "FIO1", 0)
    ljm.eWriteName(handle, "FIO4", 0)
    ljm.eWriteName(handle, "FIO2", 0)
    ljm.eWriteName(handle, "FIO0", 0)
    h = 1
    for dato in ANS:
	print ("Distancia %i : %f" % (h, dato))
	h=h+1
    print("\nStop Stream")
    ljm.eStreamStop(handle)

    # Close handle
    ljm.close(handle)
    print("Termino")


nombreArchivo = "Prueba.csv"
generador_Frecuecia (1000, 1000, nombreArchivo)
