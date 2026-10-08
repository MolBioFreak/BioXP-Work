import json, subprocess, sys
from pathlib import Path
root=Path(__file__).parent
roster=list(dict.fromkeys(json.loads((root/'native-finish-roster.json').read_text())+['test_cavro_review_regressions.py','test_native_close_contracts.py','test_bms_method_documents_connected.py','test_bms_method_full_connected.py','test_cavro_application.py','test_oem_pipette_calibration.py']))
out=root/'native-close-final-roster'; out.mkdir(exist_ok=True)
results=[]
for i,name in enumerate(roster):
    with (out/(name+'.log')).open('w') as log:
        p=subprocess.run([sys.executable,str(root/'run-native-close-offline.py'),'tests/'+name,'--junitxml='+str(out/(name+'.xml'))],stdout=log,stderr=subprocess.STDOUT)
    results.append({'file':name,'exit_code':p.returncode})
    (out/'result.json').write_text(json.dumps(results,indent=2))
    print(name,p.returncode,flush=True)
raise SystemExit(any(r['exit_code'] for r in results))
