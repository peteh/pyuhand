#/bin/bash
rm -r .venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

#python setup.py develop