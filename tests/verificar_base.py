"""Checks the recovered, hardware-validated acquisition before motor changes."""
from pathlib import Path
import hashlib
import json
import re

root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'docs/firmware_verificado_sha256.json').read_text())
baseline=root/'backups/adquisicion_validada_2026-10-31/main.cpp'
assert hashlib.sha256(baseline.read_bytes()).hexdigest()==manifest['src/main.cpp']
assert hashlib.sha256((root/'include/state_math.h').read_bytes()).hexdigest()==manifest['include/state_math.h']

def body(text,name):
    start=re.search(r'^(?:void|uint8_t) '+name+r'\([^\n]*\)\s*\{',text,re.M).start()
    brace=text.index('{',start)
    depth=1
    end=brace+1
    while depth:
        depth += (text[end]=='{')-(text[end]=='}')
        end+=1
    return text[start:end]

before=baseline.read_text()
after=(root/'src/main.cpp').read_text()
for name in ['ab','irqL','irqR','initSensorBus','serviceSample','serviceTx']:
    assert body(before,name)==body(after,name),f'Acquisition function changed: {name}'
for text in (before,after):
    assert 'if(!Serial)' not in text and 'if(!Serial||' not in text
header=lambda text:re.search(r'const char \*header=(.*);',text).group(1)
assert header(before)==header(after),'27-column CSV schema changed'
print('PASS: exact recovered baseline hash, estimator, encoders, I2C, USB and CSV preserved.')
