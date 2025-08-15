import uvicorn
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a FastAPI worker for parallel chess evaluation.")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind to.")
    parser.add_argument("--port", type=int, default=3000, help="Port to bind to.")
    args = parser.parse_args()

    # Импортируем FastAPI приложение
    from drone.fast_api_priem import app as worker_app

    print(f"Starting parallel worker server on {args.host}:{args.port}...")
    
    uvicorn.run(
        worker_app, 
        host=args.host, 
        port=args.port,
        log_level="info"
    )
