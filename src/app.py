#!/usr/bin/env python3
"""GTK 3 desktop interface for the serial robot generator."""

import json
import math
from pathlib import Path
import threading
import sys
    
# GTK setup
try:
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, GLib
except (ImportError, ValueError):
    sys.exit('GTK 3/PyGObject is missing. See README.md for installation instructions.')

# URDF generation utility
from generator import default_config, generate, export, space_size, AXES

class Window(Gtk.Window):
    def __init__(self):
        super().__init__(title='Serial Robot Generator')

        # GTK window
        self.set_default_size(820, 820)
        self.connect('delete-event', self.close)

        self.rows = []
        self.model = None
        self.busy = False
        self.cancel_event = threading.Event()
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10, margin=16)
        self.add(root)
        title = Gtk.Label(xalign=0)
        title.set_markup('<big><b>Serial Robot Generator</b></big>')
        root.pack_start(title, False, False, 0)
        root.pack_start(Gtk.Label(label='Configure a serial chain -> export a reproducible batch', xalign=0), False, False, 0)
        self.controls = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        root.pack_start(self.controls, True, True, 0)
        toolbar = Gtk.Box(spacing=8)
        self.controls.pack_start(toolbar, False, False, 0)
        self.pattern = Gtk.Entry(text='PXX', width_chars=15)
        toolbar.pack_start(Gtk.Label(label='Joint pattern'), False, False, 0)
        toolbar.pack_start(self.pattern, False, False, 0)
        self.button(toolbar, 'Apply pattern', self.apply_pattern)
        self.button(toolbar, '+ Joint', lambda *_: self.add_row())
        self.button(toolbar, 'Load settings', self.load)
        self.button(toolbar, 'Save settings', self.save)
        hint = Gtk.Label(label='Type: R / P / X (either)    Axis & link direction: x / y / z / xy / * (any)\nLength & mass: 0.4  |  0.2,0.4,0.6  |  0.2:0.6:0.1  |  uniform(0.2,0.6)', xalign=0)
        self.controls.pack_start(hint, False, False, 0)
        self.grid = Gtk.Grid(column_spacing=8, row_spacing=6, margin=4)
        scroll = Gtk.ScrolledWindow()
        scroll.set_min_content_height(170)
        scroll.add(self.grid)
        self.controls.pack_start(scroll, True, True, 0)
        self.settings = {}
        options = Gtk.Grid(column_spacing=12, row_spacing=6)
        self.controls.pack_start(options, False, False, 0)
        defaults = default_config()
        for i, (key, label) in enumerate((('radius','Link radius (m)'), ('base_height','Base height (m)'),
                    ('revolute_limit','R limit ± (rad)'), ('effort','Max effort (Nm / N)'), ('velocity','Max speed (rad/s / m/s)'))):
            entry = Gtk.Entry(text=str(defaults[key]), width_chars=9)
            self.settings[key] = entry
            row, col = divmod(i, 3)
            options.attach(Gtk.Label(label=label, xalign=0), col*2, row, 1, 1)
            options.attach(entry, col*2+1, row, 1, 1)
        batch = Gtk.Box(spacing=8)
        self.controls.pack_start(batch, False, False, 0)
        self.mode = Gtk.ComboBoxText()
        self.mode.append('random', 'Random unique samples'); self.mode.append('all', 'All discrete combinations')
        self.mode.set_active_id('random')
        batch.pack_start(self.mode, False, False, 0)
        self.count = Gtk.Entry(text='50', width_chars=6)
        self.seed = Gtk.Entry(text='42', width_chars=8)
        for label, entry in [('Count', self.count), ('Seed', self.seed)]:
            batch.pack_start(Gtk.Label(label=label), False, False, 0)
            batch.pack_start(entry, False, False, 0)
        self.button(batch, 'Count combinations', self.count_space)
        footer = Gtk.Box(spacing=8)
        root.pack_start(footer, False, False, 0)
        self.generate_button = self.button(footer, 'Export URDF batch…', self.start_export)
        self.cancel_button = self.button(footer, 'Cancel batch', lambda *_: self.cancel_event.set())
        self.cancel_button.set_sensitive(False)
        self.status = Gtk.Label(label='Ready', xalign=0)
        self.status.set_line_wrap(True)
        footer.pack_start(self.status, True, True, 0)
        self.set_config(defaults)
        self.show_all()

    def button(self, box, text, callback):
        button = Gtk.Button(label=text)
        button.connect('clicked', callback)
        box.pack_start(button, False, False, 0)
        return button

    def rebuild(self):
        for child in self.grid.get_children(): self.grid.remove(child)
        for col, text in enumerate(('#', 'Type', 'Joint axis', 'Link direction', 'Length (m)', 'Mass (kg)', 'Order / remove')):
            self.grid.attach(Gtk.Label(label=text, xalign=0), col, 0, 1, 1)
        for i, entries in enumerate(self.rows):
            self.grid.attach(Gtk.Label(label=str(i+1)), 0, i+1, 1, 1)
            for col, entry in enumerate(entries, 1): self.grid.attach(entry, col, i+1, 1, 1)
            actions = Gtk.Box(spacing=3)
            self.button(actions, '↑', lambda _, index=i: self.move(index, -1))
            self.button(actions, '↓', lambda _, index=i: self.move(index, 1))
            self.button(actions, '-', lambda _, index=i: self.remove(index))
            self.grid.attach(actions, 6, i+1, 1, 1)
        self.grid.show_all()

    def add_row(self, row=None):
        if len(self.rows) >= 30:
            return self.error('Maximum 30 joints.')
        row = row or dict(type='X', axis='*', direction='x', length='0.3', mass='1')
        self.rows.append([Gtk.Entry(text=str(row[k]), width_chars=5 if i<3 else 17)
                          for i,k in enumerate(('type','axis','direction','length','mass'))])
        self.rebuild()

    def move(self, i, delta):
        j = i+delta
        if 0 <= j < len(self.rows):
            self.rows[i], self.rows[j] = self.rows[j], self.rows[i]
            self.rebuild()

    def remove(self, i):
        if len(self.rows)>1:
            self.rows.pop(i); self.rebuild()

    def apply_pattern(self, *_):
        pattern = self.pattern.get_text().strip().upper()
        if not 1 <= len(pattern) <= 30 or any(t not in 'RPX' for t in pattern):
            return self.error('Use 1–30 characters: R, P or X. Example: PXX.')
        old = self.get_rows()
        self.rows = []
        for i,t in enumerate(pattern):
            row = old[i] if i<len(old) else dict(axis='*', direction='x', length='0.3', mass='1')
            row['type']=t
            self.add_row(row)

    def get_rows(self):
        return [dict(zip(('type','axis','direction','length','mass'), [e.get_text() for e in row])) for row in self.rows]

    def config(self):
        return dict(rows=self.get_rows(), mode=self.mode.get_active_id(), count=int(self.count.get_text()),
                    seed=int(self.seed.get_text()), **{k:float(e.get_text()) for k,e in self.settings.items()})

    def set_config(self, config):
        space_size(config)
        if config['mode'] not in ('all','random'): raise ValueError('Unknown mode.')
        int(config['count']); int(config['seed'])
        self.rows=[]
        for row in config['rows']: self.add_row(row)
        for key, entry in self.settings.items(): entry.set_text(str(config[key]))
        self.mode.set_active_id(config['mode'])
        self.count.set_text(str(config['count'])); self.seed.set_text(str(config['seed']))
        self.pattern.set_text(''.join(r['type'].upper() if r['type'].upper() in ('R','P') else 'X' for r in config['rows']))
        self.model=None; 
        # self.area.queue_draw()

    def error(self, message):
        dialog=Gtk.MessageDialog(transient_for=self, modal=True, message_type=Gtk.MessageType.ERROR,
                                 buttons=Gtk.ButtonsType.CLOSE, text=str(message))
        dialog.run(); dialog.destroy()

    def choose(self, title, action):
        dialog=Gtk.FileChooserDialog(title=title, transient_for=self, action=action)
        dialog.add_buttons('Cancel', Gtk.ResponseType.CANCEL, 'Select', Gtk.ResponseType.OK)
        if action==Gtk.FileChooserAction.SAVE:
            dialog.set_current_name('config.json'); dialog.set_do_overwrite_confirmation(True)
        result=dialog.get_filename() if dialog.run()==Gtk.ResponseType.OK else None
        dialog.destroy()
        return result

    def save(self, *_):
        try:
            config=self.config(); space_size(config)
            path=self.choose('Save generator settings', Gtk.FileChooserAction.SAVE)
            if path: Path(path).write_text(json.dumps(config, indent=2)+'\n')
        except Exception as e: self.error(e)

    def load(self, *_):
        path=self.choose('Load generator settings JSON', Gtk.FileChooserAction.OPEN)
        if path:
            try: self.set_config(json.loads(Path(path).read_text())); # self.preview()
            except Exception as e: self.error(e)

    def count_space(self, *_):
        try:
            size=space_size(self.config())
            self.status.set_text('Continuous parameter space; use random sampling.' if size is None else f'{size:,} discrete combinations')
        except Exception as e: self.error(e)

   
    def start_export(self, *_):
        try:
            config=self.config(); next(generate(config))
        except Exception as e: return self.error(e)
        path=self.choose('Select parent output folder', Gtk.FileChooserAction.SELECT_FOLDER)
        if not path: return
        self.busy=True; self.cancel_event.clear()
        self.controls.set_sensitive(False); self.generate_button.set_sensitive(False); self.cancel_button.set_sensitive(True)
        self.status.set_text('Generating…')
        def progress(n):
            if n%25==0: GLib.idle_add(self.status.set_text, f'Exported {n} robots…')
        def worker():
            try:
                folder,count=export(config,path,progress,self.cancel_event.is_set)
                message=f'{"Cancelled; kept" if self.cancel_event.is_set() else "Exported"} {count} robots in {folder}'
            except Exception as e: message=f'Export failed: {e}'
            GLib.idle_add(self.finished,message)
        threading.Thread(target=worker,daemon=True).start()

    def finished(self,message):
        self.busy=False; self.controls.set_sensitive(True); self.generate_button.set_sensitive(True)
        self.cancel_button.set_sensitive(False); self.status.set_text(message)
        return False

    def close(self,*_):
        if self.busy:
            self.cancel_event.set(); self.status.set_text('Cancelling… close again after export stops.'); return True
        Gtk.main_quit(); return False

if __name__=='__main__':
    Window()
    Gtk.main()
