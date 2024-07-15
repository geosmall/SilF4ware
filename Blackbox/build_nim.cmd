REM nim -d:release --opt:size --passC:-flto --passL:-flto --passL:-s c txt2bbl.nim && strip -s txt2bbl.exe
nim -d:release --opt:size --passC:-flto --passL:-s --passL:-flto c txt2bbl.nim
dir