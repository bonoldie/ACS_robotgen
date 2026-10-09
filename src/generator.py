"""Dependency-free serial manipulator generator. SI units; Python >= 3.10."""
from dataclasses import dataclass, asdict
from itertools import product
from pathlib import Path
import argparse
import json
import math
import random
import re
import xml.etree.ElementTree as ET
from datetime import datetime
import itertools
from tempfile import mkdtemp
 
AXES = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
MAX_BATCH = 10000

PRISMATIC_COLOR = '0.2 0.55 0.95 1'
PRISMATIC_JOINT_COLOR = '0 0 1.0 1'
REVOLUTE_COLOR = '0.9 0.35 0.22 1'
REVOLUTE_JOINT_COLOR = '1.0 0 0 1'

@dataclass(frozen=True)
class Domain:
    values: tuple = ()
    bounds: tuple = ()
    
    def sample(self, rng):
        return rng.uniform(*self.bounds) if self.bounds else rng.choice(self.values)

def numeric(text):
    """Validate scalar, comma list, lo:hi:step, or uniform(lo,hi)/rand(lo,hi)."""
    
    text = str(text).strip().lower()
    match = re.fullmatch(r'(?:uniform|rand)\(([^,]+),([^,]+)\)', text)
    
    if match:
        # uniform/random choices from interval case
        lo, hi = map(float, match.groups())
        if not (math.isfinite(lo) and math.isfinite(hi) and 0 < lo <= hi):
            raise ValueError('Random bounds must be finite, positive and increasing.')
        return Domain(values=(lo,)) if lo == hi else Domain(bounds=(lo, hi))
    
    if ':' in text:
        # range interval case
        parts = text.split(':')
        
        if len(parts) != 3:
            raise ValueError('Use lo:hi:step or uniform(lo,hi).')
        
        lo, hi, step = map(float, parts)
        
        if not all(map(math.isfinite, (lo, hi, step))) or lo <= 0 or hi < lo or step <= 0:
            raise ValueError('Range requires 0 < lo <= hi and step > 0.')
        
        n = math.floor((hi-lo)/step + 1e-10) + 1
        
        # harcoded limit of 10k values per range
        if n > 10000:
            raise ValueError('At most 10,000 values per range.')
        
        values = tuple(lo + i*step for i in range(n))
    else:
        # list case
        values = tuple(float(v.strip()) for v in text.strip('[]').split(','))
        
    # masses/lengths validation
    if not values or any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Lengths and masses must be finite and greater than zero.')
    
    return Domain(tuple(dict.fromkeys(values)))

def choices(text, allowed):
    text = text.strip().lower()
    
    if text in ('*', '?', 'random', 'rand') or (allowed == 'rp' and text == 'x'):
        text = allowed
    
    values = tuple(dict.fromkeys(text.replace(',', '').replace(' ', ''))) 
    
    if not values or any(v not in allowed for v in values):
        raise ValueError(f'Expected choices from {allowed}, or *.')
    
    return Domain(values)

def default_config():
    return {
        'rows': [
            {'type': t, 'axis': '*', 'direction': d, 'length': '0.2,0.4', 'mass': '1'} for t, d in zip('PXX', 'xyz')
        ],
        'mode': 'random', 
        'count': 50, 
        'seed': 42, 
        'radius': 0.02,
        'base_height': 0.15, 
        'revolute_limit': math.pi,
        'effort': 100.0, 
        'velocity': 1.0
    }

def domains(config):
    rows = config['rows']
    
    # joints count check (hardcoded to 30 joints max)
    if not 1 <= len(rows) <= 30:
        raise ValueError('Use between 1 and 30 joints.')
    
    # bounded/nonzero check
    for key in ('radius', 'base_height', 'revolute_limit', 'effort', 'velocity'):
        value = float(config[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive.')
    
    result = []
    for i, row in enumerate(rows, 1):
        try:
            # For prismatic joints we override the direction since the joint should accomodate the link movement
            if 'direction' not in row.keys():
                row['direction'] = row['axis']
            
            result.extend((choices(row['type'], 'rp'), choices(row['axis'], 'xyz'), choices(row['direction'], 'xyz'), numeric(row['length']), numeric(row['mass'])))
        except (ValueError, KeyError) as e:
            raise ValueError(f'Joint {i}: {e}') from e
    return result

def space_size(config):
    """Gets the total number of combinations of parameters for a given configuration"""
    
    # each row in the config has multiple domains
    ds = domains(config)

    return None if any(d.bounds for d in ds) else math.prod(len(d.values) for d in ds)

def decode(index, ds):
    vals = []
    
    for d in reversed(ds):
        index, digit = divmod(index, len(d.values))
        vals.append(d.values[digit])
        
    return list(reversed(vals))

def generate(config):
    ds = domains(config)
    size = space_size(config)
    
    rng = random.Random(int(config['seed']))
    
    mode = config['mode']
    
    if mode == 'all':
        if size is None:
            raise ValueError('Exhaustive mode needs discrete values. Replace uniform(lo,hi) with lo:hi:step.')
        if size > MAX_BATCH:
            raise ValueError(f'{size:,} combinations exceeds {MAX_BATCH:,}. Use random mode or narrow the choices.')
        samples = product(*(d.values for d in ds))
    elif mode == 'random':
        count = int(config['count'])
        if not 1 <= count <= MAX_BATCH:
            raise ValueError(f'Batch size must be 1–{MAX_BATCH}.')
        if size is not None:
            # Floyd sampling works even for spaces larger than sys.maxsize.
            selected, indices = set(), []
            for j in range(size-min(size,count), size):
                t = rng.randrange(j+1)
                value = j if t in selected else t
                selected.add(value); 
                indices.append(value)
            
            rng.shuffle(indices)
            samples = (decode(index, ds) for index in indices)
        else:
            samples = ([d.sample(rng) for d in ds] for _ in range(count))
    else:
        raise ValueError(f'Unknown generation mode.')
    
    for vals in samples:
        yield [dict(zip(('type', 'axis', 'direction', 'length', 'mass'), vals[i:i+5])) for i in range(0, len(vals), 5)]

def fmt(v):
    return format(float(v), '.12g')

def vec(v):
    return ' '.join(map(fmt, v))

def scale(axis, value):
    return tuple(value*x for x in AXES[axis])

def urdf(model, config, name='manipulator'):
    root = ET.Element('robot', name=name)
    
    def node(parent, tag, **attrs):
        return ET.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})
    
    def link_joint(name, type, length, mass, direction, radius, color):
        link = node(root, 'link', name=name)
        center = vec(scale(direction, length/2))
        rpy = {'x':f'0 {math.pi/2} 0', 'y':f'{-math.pi/2} 0 0', 'z':'0 0 0'}[direction]
        for tag in ('visual', 'collision'):
            part = node(link, tag)
            node(part, 'origin', xyz=center, rpy=rpy)
            
            if type == 'r':
                node(node(part, 'geometry'), 'cylinder', radius=fmt(radius*0.95), length=fmt(length))
            else:
                node(node(part, 'geometry'), 'box', size=vec([radius*1.8, radius*1.8, length]))
            
            if tag == 'visual':
                node(node(part, 'material', name=name+'_color'), 'color', rgba=color)
        
        inertial = node(link, 'inertial')
        node(inertial, 'origin', xyz=center, rpy='0 0 0')
        node(inertial, 'mass', value=fmt(mass))
        
        # transverse = mass*(3*radius**2 + length**2)/12
        # inertia = [transverse]*3
        # inertia['xyz'.index(direction)] = mass*radius**2/2
        # node(inertial, 'inertia', ixx=fmt(inertia[0]), iyy=fmt(inertia[1]), izz=fmt(inertia[2]), ixy='0', ixz='0', iyz='0')
        node(inertial, 'inertia', ixx='0', iyy='0', izz='0', ixy='0', ixz='0', iyz='0')

        return link
        
    # base link    
    node(root, 'link', name='world')
    parent_link_node = link_joint('base_link', 'p', float(config['base_height']), 1.0, 'z', float(config['radius'])*2, '0.3 0.35 0.4 1')
    fixed = node(root, 'joint', name='world_fixed', type='fixed')
    node(fixed, 'parent', link='world'); node(fixed, 'child', link='base_link')
    offset = (0, 0, float(config['base_height']))
    parent = 'base_link'
    
    for i, row in enumerate(model, 1):
        child = f'link_{i}'
        joint_type = 'revolute' if row['type']=='r' else 'prismatic'
        
        row_link_node = link_joint(child, row['type'], row['length'], row['mass'], row['direction'] if row['type'] == 'r' else row['axis'], float(config['radius']), REVOLUTE_COLOR if row['type'] == 'r' else PRISMATIC_COLOR)
        joint = node(root, 'joint', name=f'joint_{i}', type=joint_type)
        
        node(joint, 'parent', link=parent)
        node(joint, 'child', link=child)
        node(joint, 'origin', xyz=vec(offset), rpy='0 0 0')
        node(joint, 'axis', xyz=vec(AXES[row['axis']]))
        
        limit = float(config['revolute_limit']) if row['type']=='r' else float(row['length'])
        node(joint, 'limit', lower=fmt(-limit), upper=fmt(limit) if row['type']=='r' else '0', effort=fmt(config['effort']), velocity=fmt(config['velocity']))
        node(joint, 'dynamics', damping='0', friction='0')
        
        
        # Additional visual to identify joint type
        joint_type_visual = node(parent_link_node, 'visual')
                
        rpy = {'x':f'0 {math.pi/2} 0', 'y':f'{-math.pi/2} 0 0', 'z':'0 0 0'}[row['axis']]
        
        node(joint_type_visual, 'origin', xyz=vec(offset), rpy=rpy)
        
        if row['type'] == 'r':
            node(node(joint_type_visual, 'geometry'), 'cylinder', radius=config['radius'] * 1.3, length='0.04')
        else:   
            node(node(joint_type_visual, 'geometry'), 'box', size=vec([config['radius']*2, config['radius']*2, config['radius']*3]))              
        
        node(node(joint_type_visual, 'material', name = f"{child}_joint_color"), 'color', rgba = REVOLUTE_JOINT_COLOR if row['type'] == 'r' else PRISMATIC_JOINT_COLOR)

        parent_link_node = row_link_node 
        parent, offset = child, scale(row['direction'] if row['type'] == 'r' else row['axis'], row['length'])

    
    # Adding EE link
    node(root, 'link', name='ee')
    
    joint = node(root, 'joint', name=f'ee_joint', type='fixed')
      
    node(joint, 'parent', link=parent) 
    node(joint, 'child', link='ee')
    node(joint, 'origin', xyz=vec(offset), rpy='0 0 0')
    
    ET.indent(root, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'

def export(config, destination, progress=lambda n: None, cancelled=lambda: False):
    # validate before creating an output folder.
    iterator = generate(config)
    first = next(iterator)
    
    destination = Path(destination).expanduser()
    destination.mkdir(parents=True, exist_ok=True)
    
    folder = Path(mkdtemp(prefix=datetime.now().strftime('robots_%Y%m%d_%H%M%S_'), dir=destination))
    
    # dark art of python to write to file
    (folder / 'config.json').write_text(json.dumps(config, indent=2)+'\n')
    
    records = []
    
    try:
        for i, model in enumerate(itertools.chain([first], iterator), 1):
            if cancelled():
                break
            label = ''.join(r['type'].upper() for r in model) + '_' + ''.join(r['axis'] for r in model)
            filename = f'{i:05d}_{label}.urdf'
            (folder / filename).write_text(urdf(model, config, f'robot_{i:05d}_{label}'))
            
            records.append({'file': filename, 'joints':model})
            progress(i)
    finally:
        (folder / 'manifest.json').write_text(json.dumps({'config':config, 'count':len(records), 'models':records}, indent=2)+'\n')
    
    return folder, len(records)

if __name__ == '__main__':
    # args setup
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, default=Path('generated'))
    args = parser.parse_args()
    
    # config file is loaded from disk, fallback to defaults
    config = json.loads(args.config.read_text()) if args.config else default_config()
    
    try:
        folder, count = export(config, args.output, progress= lambda n: print(f'Robot #{n} generated.'))
        print(f'Exported {count} robots to {folder}')
    except (ValueError, KeyError, OSError) as e:
        parser.exit(1, f'Error: {e}\n')
