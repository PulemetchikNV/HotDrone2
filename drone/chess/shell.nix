{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.stockfish
    pkgs.python3
    pkgs.python3Packages.chess
    pkgs.python3Packages.fastapi
    pkgs.python3Packages.uvicorn
    pkgs.python3Packages.httpx
    pkgs.python3Packages.psutil
  ];
}
