import uvicorn
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a FastAPI worker for the chess engine.")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind to. Use 0.0.0.0 for remote access.")
    parser.add_argument("--port", type=int, required=True, help="Port to bind to.")
    # Добавляем аргумент для управления перезагрузкой, но по умолчанию он False
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload for development.")
    args = parser.parse_args()

    # Импортируем приложение из нового файла
    from drone.fast_api_priem import app as worker_app

    print(f"Starting worker server on {args.host}:{args.port}...")
    
    # Запускаем uvicorn программно для лучшего контроля
    uvicorn.run(
        worker_app, 
        host=args.host, 
        port=args.port,
        reload=args.reload,
        log_level="info"
    )
