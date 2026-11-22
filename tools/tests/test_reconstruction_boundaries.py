"""Architecture contracts: numerical layers cannot acquire CAD/UI dependencies."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / 'core/geometric_reconstruction'
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)', re.M)


class ReconstructionBoundaryTests(unittest.TestCase):
    def test_numerical_dependencies(self):
        allowed = {
            'data': {'data'},
            'preprocessing': {'data', 'preprocessing'},
            'segmentation': {'data', 'segmentation'},
            'surface': {'data', 'segmentation', 'preprocessing', 'surface', 'settings'},
        }
        for layer, dependencies in allowed.items():
            for directory in [CORE/'src'/layer, CORE/'include/reconstruction'/layer]:
                for path in directory.rglob('*'):
                    if path.suffix not in {'.h', '.cpp'}:
                        continue
                    # This historical type header forwards to recognition only.
                    if path == CORE/'include/reconstruction/data/ReconstructionCandidate.h':
                        continue
                    for include in INCLUDE.findall(path.read_text(encoding='utf-8')):
                        with self.subTest(path=str(path.relative_to(ROOT)), include=include):
                            self.assertFalse(include.startswith(('vtk', 'Qt', 'TopoDS', 'BRep', 'QThread', 'QCoreApplication')))
                            if include.startswith('modeling/'):
                                self.assertEqual(include, 'modeling/Geometry.h')
                            if include.startswith('reconstruction/'):
                                self.assertIn(include.split('/')[1], dependencies)
                            if include.startswith('reconstruction/settings/'):
                                self.assertEqual(include, 'reconstruction/settings/SurfaceMeshingSettings.h')
                            self.assertNotIn('recognition/', include)
                            self.assertNotIn('pipeline/', include)
                            self.assertNotIn('parts/', include)

    def test_source_ownership(self):
        sources = re.findall(r'^\s+(src/\S+\.cpp)\s*$', (CORE/'cmake/ReconstructionModules.cmake').read_text(), re.M)
        self.assertEqual(len(sources), len(set(sources)), 'Each source must have exactly one owning target')
        actual = {p.relative_to(CORE).as_posix() for directory in
                  ['data', 'preprocessing', 'segmentation', 'surface', 'recognition', 'parts', 'pipeline', 'modeling', 'settings', 'testing', 'solid_inference']
                  for p in (CORE/'src'/directory).rglob('*.cpp')}
        self.assertEqual(set(sources), actual)

    def test_internal_consumers_use_canonical_headers(self):
        old = re.compile(r'^reconstruction/(analysis/|detection/|recognition/(CompoundRecognizer|AssemblyReconstruction|ProjectionExtrusion|ProjectionSettings|GuidedSolidRecognizer)\.h)')
        for directory in [CORE/'src', ROOT/'apps/ParamCAD Studio/src', ROOT/'tools/validation/native']:
            for path in directory.rglob('*'):
                if path.suffix not in {'.h', '.cpp'}:
                    continue
                for include in INCLUDE.findall(path.read_text(encoding='utf-8')):
                    self.assertFalse(old.match(include), f'{path}: compatibility include {include}')
                    # Stage-contract tools intentionally exercise private stages;
                    # production algorithms and adapters must use the public API.
                    if path.is_relative_to(CORE/'src') and '/preprocessing/' not in path.as_posix():
                        self.assertFalse(include.startswith('preprocessing/detail/'), f'{path}: preprocessing private include')

    def test_pipeline_uses_feature_interface(self):
        text = (CORE/'src/pipeline/recovery/RepairTransaction.cpp').read_text(encoding='utf-8')
        self.assertIn('reconstruction/recognition/CandidateMaterial.h', text)
        self.assertNotIn('MixedInternal.h', text)


if __name__ == '__main__':
    unittest.main()
