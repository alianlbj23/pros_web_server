# pros_web_server

## Clone
```bash
git clone --recurse-submodules <repo_url>
```

## Setup
Run the setup script to create a web server that automatically starts on device boot:
```bash
./setup_gunicorn_service.sh
```

This will:
- Create a Python virtual environment
- Install all required dependencies from `requirements.txt`
- Create a systemd service that runs the web server on port 6000
- Configure the web server to automatically start when the device boots up
- Enable the service to restart automatically if it crashes

## Usage

### Manual server start (alternative to service)
```bash
gunicorn --workers 4 --bind 0.0.0.0:6000 main:app
```

### Send signal to start service
```bash
curl http://localhost:6000/run-script/star_car
# or from remote
curl http://<ip>:6000/run-script/star_car
```

### Stop service
```bash
curl http://localhost:6000/run-script/<service_name>_stop
```

### Check service status
```bash
sudo systemctl status gunicorn_web_server
```

## Uninstall
To completely remove the service and clean up all files:
```bash
./uninstall_gunicorn_service.sh
```

This will:
- Stop and disable the systemd service
- Remove the service file
- Delete the virtual environment
- Clean up all configurations
## Client
https://github.com/alianlbj23/pros_web_client