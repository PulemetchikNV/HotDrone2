{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  # buildInputs are the packages available in the shell
  buildInputs = [
    # Use Python 3.12
    pkgs.python312

    # Add the Flask package for Python 3.12
    pkgs.python312Packages.flask
    
    # Add the httpx package for making HTTP requests
    pkgs.python312Packages.httpx
  ];
}
