# Gemelo digital del enjambre ACO (PyBullet sin ventana + video en el navegador)
FROM python:3.11-slim
RUN apt-get update && apt-get install -y --no-install-recommends libglib2.0-0 && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY gemelo.py .
ENV AP_IP=192.168.4.1 PUERTO_WEB=8080 PYTHONUNBUFFERED=1
EXPOSE 8080
CMD ["python", "gemelo.py"]
