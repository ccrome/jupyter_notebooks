import numpy as np
import scipy as sp

from bokeh.io import curdoc
from bokeh.layouts import row, column
from bokeh.models import ColumnDataSource, Range1d
from bokeh.models.widgets import Slider, TextInput
from bokeh.plotting import figure

def compute_distances(a, b):
    """a and b are 2d vecotrs of shape (n, 3) where n is the number of items, and 3 is for xyz dimensions
    So, if we have a = (n, 3) and b = (m, 3), then the output will contain
    an array of shape (n, m) and each element will be the distance from item a[x], b[y]"""
    aa = a[np.newaxis, :, :]
    bb = b[:, np.newaxis, :]
    ss = (aa - bb)**2
    s = np.sqrt(np.sum(ss, axis=2))
    return s
    
def compute_response(listener_distance, angles, speaker_locations, freqs):
    angles_rad = np.deg2rad(angles)
    listener_positions = np.zeros((len(angles), 3))
    listener_positions[:, 0] = np.cos(angles_rad)
    listener_positions[:, 1] = np.sin(angles_rad)
    listeners = listener_positions * listener_distance
    speakers = np.array(speaker_locations)

    distances = compute_distances(speakers, listeners)  # mm
    # Consider distances[0, 0] to be amplitude = 1.0, and amplitude drops as 1/r**2
    amplitudes = 1.0/(distances / distances[0, 0])**2
    distances = distances[np.newaxis, :, :]
    freqs1 = freqs[:, np.newaxis, np.newaxis]
    phase = freqs1 * distances / c * 2 * np.pi   # cycles/sec * mm * sec / mm * rad/cycle = rad
    phasor = np.cos(phase) + 1j*np.sin(phase)
    amplitudes = amplitudes[np.newaxis, :, :]
    mag = np.abs(np.sum(phasor * amplitudes, axis=2))
    mag = mag / len(speakers)
    return mag, listeners, speakers

def setup_controls():
    controls = {}
    controls["distance_apart"]    = Slider(title="Speaker distance(mm)", value=30.0, start=1.0, end=300, step=1)
    controls["listener_distance"] = Slider(title="Listener distance(mm)", value=1000.0, start=1.0, end=10000, step=1)
    controls["listener_angle"]    = Slider(title="Listener angle (deg)", value=30, start=0, end=360, step=1)
    return controls

def update_data(attrname, old, new):
    # Get current values
    d = controls["distance_apart"].value
    l = controls["listener_distance"].value
    a = controls["listener_angle"].value
    sl = np.array([[0.0,  d/2., 0],
                   [0.0, -d/2., 0]])
    mag, listeners, speakers = compute_response(listener_distance = l,
                           angles = np.array([a], dtype=float),
                           speaker_locations = sl,
                           freqs = freqs)
    x = freqs
    y = 20*np.log10(np.abs(mag[:,0]))
    source.data = dict(x = x, y = y)
    graphic_source_speakers.data = dict(x = speakers[:, 0], y = speakers[:, 1])
    graphic_source_listeners.data = dict(x=[listeners[0, 0],], y=[listeners[0, 1],])
    x = np.transpose(np.array([speakers[:,0], np.tile(listeners[:,0], speakers.shape[0])]))
    y = np.transpose(np.array([speakers[:,1], np.tile(listeners[:,1], speakers.shape[0])]))
    graphic_source_lines.data = dict(x = x, y = y)
    graphic.x_range=Range1d(-l-20, l+20)
    graphic.y_range=Range1d(-l-20, l+20)

# All measurements in mm
angles = np.linspace(0, 90, 10, endpoint=True)
num_freqs = 5000
freqs  = np.logspace(np.log10(20), np.log10(20000), num_freqs, endpoint=True, base=10.0)
c = 343000.0 # mm/sec
controls = setup_controls()

speaker_locations = [[0,  25.4/2, 0],
                     [0, -25.4/2, 0]]


mag = compute_response(listener_distance = 1000, angles=angles, speaker_locations=speaker_locations, freqs=freqs)

source = ColumnDataSource(data = dict(x=[0.0], y= [0.0]))
graphic_source_speakers = ColumnDataSource(data = dict(x=[0], y= [0]))
graphic_source_listeners = ColumnDataSource(data = dict(x=[0], y=[0]))
graphic_source_lines = ColumnDataSource(data = dict(x=[0], y=[0]))

plot = figure(plot_height= 400,
              plot_width = 400,
              title      = "Interference between sources",
              tools      = "crosshair,pan, reset, save, wheel_zoom",
              x_range    = [20, 20000],
              y_range    = [-60, 10],
              )
plot.yaxis.axis_label="Response (dB)"
plot.xaxis.axis_label="Frequency"
graphic = figure(plot_height= 400,
              plot_width = 400,
              title      = "Source Locations",
              tools      = "crosshair,pan, reset, save, wheel_zoom",
              )
graphic.yaxis.axis_label="y (mm)"
graphic.xaxis.axis_label="x (mm)"

cv = [controls[c] for c in controls]
[c.on_change('value', update_data) for c in cv]
    
plot.line('x', 'y', source=source, line_width=3, line_alpha=0.6)
graphic.annular_wedge(x='x', y='y', source = graphic_source_speakers, inner_radius=3, outer_radius = 30, start_angle=np.pi/2, end_angle=3*np.pi/2, color="green", alpha = 0.6)
graphic.circle('x', 'y', source = graphic_source_listeners)
graphic.multi_line('x', 'y', source=graphic_source_lines)
update_data(None, None, None)

curdoc().add_root(row(column(*cv), plot, graphic))
curdoc().title="asdf"
