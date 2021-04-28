#python3 -m twine upload --repository testpypi dist/*
python3 -m pip install --upgrade build
python3 -m pip install --upgrade twine
rm -r ./dist
python3 -m build
python3 -m twine upload dist/*
#python3 -m pip install --index-url https://test.pypi.org/simple/ --no-deps example-pkg-YOUR-USERNAME-HERE