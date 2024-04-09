from flask import Flask, jsonify, render_template
from datetime import datetime
import requests

app = Flask(__name__)


@app.route('/')
def home():
    return render_template('index.html')

@app.route('/timestamp', methods=['GET'])
def get_timestamp():
    timestamp = datetime.now().isoformat()
    # Return the timestamp as a JSON response
    return jsonify({'timestamp': timestamp})

if __name__ == '__main__':
    app.run(debug=True, port=5000, host='0.0.0.0')
