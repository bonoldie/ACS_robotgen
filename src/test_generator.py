import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from generator import *

class GeneratorTests(unittest.TestCase):
    def fixed(self):
        c=default_config()
        for row in c['rows']: row.update(axis='x', length='0.4', mass='2')
        return c

    def test_pxx_exhaustive(self):
        c=self.fixed(); c['mode']='all'
        models=list(generate(c))
        self.assertEqual({''.join(r['type'] for r in m) for m in models}, {'prr','prp','ppr','ppp'})
        self.assertEqual(space_size(c),4)

    def test_axis_and_numeric_product(self):
        c=self.fixed(); c['rows']=c['rows'][:1]
        c['rows'][0].update(axis='*', length='0.2,0.4',mass='1:3:1')
        c['mode']='all'
        self.assertEqual(len(list(generate(c))),18)

    def test_unique_reproducible_and_cap(self):
        c=default_config(); c['count']=10000
        a=list(generate(c)); b=list(generate(c))
        self.assertEqual(a,b)
        self.assertEqual(len(a),space_size(c))
        self.assertEqual(len({json.dumps(m) for m in a}),len(a))

    def test_continuous(self):
        c=self.fixed(); c['rows'][0]['length']='uniform(0.2,0.8)'
        self.assertIsNone(space_size(c))
        self.assertTrue(all(.2<=m[0]['length']<=.8 for m in generate(c)))
        self.assertEqual(list(generate(c)),list(generate(c)))
        c['mode']='all'
        with self.assertRaises(ValueError): list(generate(c))

    def test_invalid(self):
        for value in ('0','-1','nan','inf','1:0:1','1:2:0','uniform(2,1)','1:2',''):
            with self.subTest(value=value), self.assertRaises(ValueError): numeric(value)
        self.assertEqual(len(numeric('0.2:0.6:0.1').values),5)
        c=self.fixed(); c['rows'][0]['axis']='w'
        with self.assertRaises(ValueError): list(generate(c))

    def test_huge_space_small_sample(self):
        c=default_config(); c['rows']=[dict(type='X',axis='*',direction='*',length='1,2',mass='1,2') for _ in range(30)]
        c['count']=3
        self.assertEqual(len(list(generate(c))),3)
        c['mode']='all'
        with self.assertRaises(ValueError): list(generate(c))

    def test_geometry_inertia_and_chain(self):
        c=self.fixed(); model=next(generate(c)); root=ET.fromstring(urdf(model,c))
        links={link.get('name'):link for link in root.findall('link')}
        joints=root.findall('joint')
        self.assertEqual(len(joints),len(links)-1)
        children=[j.find('child').get('link') for j in joints]
        self.assertEqual(len(children),len(set(children)))
        for j in joints:
            self.assertIn(j.find('parent').get('link'),links)
            self.assertIn(j.find('child').get('link'),links)
        for i,r in enumerate(model,1):
            link=links[f'link_{i}']; axis='xyz'.index(r['direction'])
            center=list(map(float,link.find('inertial/origin').get('xyz').split()))
            self.assertAlmostEqual(center[axis],r['length']/2)
            inertia=link.find('inertial/inertia')
            diag=[float(inertia.get(k)) for k in ('ixx','iyy','izz')]
            self.assertTrue(all(v>0 for v in diag))
            self.assertAlmostEqual(diag[axis],r['mass']*c['radius']**2/2)
            next_joint=root.find(f"joint[@name='joint_{i+1}']") if i<len(model) else root.find("joint[@name='tool_fixed']")
            offset=list(map(float,next_joint.find('origin').get('xyz').split()))
            self.assertAlmostEqual(offset[axis],r['length'])
            self.assertEqual(link.find('visual/origin').get('xyz'),link.find('collision/origin').get('xyz'))

    def test_export_and_cancel(self):
        c=self.fixed()
        with tempfile.TemporaryDirectory() as tmp:
            path,n=export(c,tmp)
            self.assertEqual(n,4)
            self.assertEqual(len(list(path.glob('*.urdf'))),n)
            self.assertEqual(json.loads((path/'manifest.json').read_text())['count'],n)
            path2,n2=export(c,tmp,cancelled=lambda:True)
            self.assertNotEqual(path,path2); self.assertEqual(n2,0)
            self.assertEqual(json.loads((path2/'manifest.json').read_text())['count'],0)

if __name__=='__main__': unittest.main()
