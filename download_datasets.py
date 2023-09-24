import urllib.request
import zipfile
from pathlib import Path

# Download the dataset smatt
url = "http://vision.deis.unibo.it/~smatt/stereo/Calibration.zip"    

filehandle, _ = urllib.request.urlretrieve(url)
zip_file_object = zipfile.ZipFile(filehandle, 'r')
zip_file_object.extractall("datasets/smatt")

# Caltech Dataset
url = "https://data.caltech.edu/records/jx9cx-fdh55/files/calib_doc.zip?download=1"
filehandle, _ = urllib.request.urlretrieve(url)
zip_file_object = zipfile.ZipFile(filehandle, 'r')
zip_file_object.extractall("datasets/caltech")

# uni-erlangen
url = "https://lntweblms.e-technik.uni-erlangen.de/datacenter/fisheyedataset/Calibration.zip"
filehandle, _ = urllib.request.urlretrieve(url)
zip_file_object = zipfile.ZipFile(filehandle, 'r')
zip_file_object.extractall("datasets/fisheye")

