import boto3
from decimal import Decimal
from datetime import datetime
import struct
import json
import base64

def float_to_bytes(floats):
    """Convert a list of floats to a list of byte strings."""
    return [struct.pack('f', f) for f in floats]

def bytes_to_float(bytes_list):
    """Convert a list of byte strings to a list of floats."""
    return [struct.unpack('f', b)[0] for b in bytes_list]

def lambda_handler(event, context):
    dynamodb = boto3.resource('dynamodb')
    table = dynamodb.Table('DeviceWeights')
    iot_client = boto3.client('iot-data', region_name='eu-north-1')

    if not isinstance(event, str):
        print("Error: Event data is not in the expected string format.")
        return {
            'statusCode': 400,
            'body': 'Bad request: Event data format is incorrect.'
        }

    try:
        incoming_weights_bytes = bytes(event, 'utf-8').decode('unicode_escape').encode('latin-1')
    except Exception as e:
        print(f"Error decoding the byte string: {str(e)}")
        return {
            'statusCode': 500,
            'body': f'Error decoding byte string: {str(e)}'
        }

    current_time = datetime.utcnow().isoformat()
    incoming_weights = bytes_to_float([incoming_weights_bytes[i:i+4] for i in range(0, len(incoming_weights_bytes), 4)])
    print("INCOMESTING WEIGHTS:")
    print(incoming_weights)

    scan_response = table.scan()
    items = scan_response['Items']

    if items:
        existing_item = items[0]
        print("existing_item:")
        print(existing_item)
        if 'Weights' in existing_item and isinstance(existing_item['Weights'], list):
            existing_weights = [float(w) for w in existing_item['Weights']]
            average_weights = [(incoming_weights[i] + existing_weights[i]) / 2 for i in range(min(len(incoming_weights), len(existing_weights)))]
        else:
            average_weights = incoming_weights
    else:
        average_weights = incoming_weights

    print("AVERAGE WEIGHTS:")
    print(average_weights)

    average_weights_decimal = [Decimal(str(w)) for w in average_weights]
    response = table.update_item(
        Key={'DeviceId': 'global'},
        UpdateExpression='SET #ts = :val, #timestamp = :time ADD #sc :inc',
        ExpressionAttributeNames={
            '#ts': 'Weights',
            '#timestamp': 'Timestamp',
            '#sc': 'StepCounter'
        },
        ExpressionAttributeValues={
            ':val': average_weights_decimal,
            ':time': current_time,
            ':inc': Decimal('1')
        },
        ReturnValues="UPDATED_NEW"
    )
    
    average_weights_bytes = b''.join(float_to_bytes(average_weights))
    iot_client.publish(
        topic='test/globalweights',
        payload= average_weights_bytes
    )
    print("Publish")
    

    return {
        'statusCode': 200,
        'body': 'Processed successfully'
    }
