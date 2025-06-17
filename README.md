# pros_web_server
## clone
```
git clone --recurse-submodules <repo_url>
```
## Usage
Open server
```shell
gunicorn --workers 4 --bind 0.0.0.0:5000 main:app
```
Send signal to open service
```
curl http://localhost:5000/run-script/star_car
or
curl http://<ip>:5000/run-script/star_car
```
Close service
```
curl http://localhost:5000/run-script/<service_name>_stop
```
