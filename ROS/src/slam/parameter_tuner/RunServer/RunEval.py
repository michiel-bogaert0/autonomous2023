import ast
import json
import os
import shutil
from collections import Counter

import numpy as np
import pandas as pd
import requests

direct = "../tuner_data"
directold = direct + "/old"
file = ""
fname = ""

if not os.path.isdir(direct):
    os.makedirs(direct)
if not os.path.isdir(directold):
    os.makedirs(directold)


def readFromFile():
    for filename in os.listdir(direct):
        if filename.endswith(".csv"):
            fname = filename
            file = os.path.join(direct, filename)
    df = pd.read_csv(
        file,
        dtype={
            "Simulation": str,
            "Parameters": str,
            "Duration": str,
            "StopReason": str,
            "avgDistanceToConeSLAM": str,
            "": str,
        },
    )
    shutil.move(file, os.path.join(directold, fname))

    df["Parameters"] = df["Parameters"].apply(lambda x: ast.literal_eval(x))
    df["avgDistanceToConeSLAM"] = df["avgDistanceToConeSLAM"].apply(
        lambda x: ast.literal_eval(x)
    )
    df["labelsConesSlam"] = df["labelsConesSlam"].apply(lambda x: ast.literal_eval(x))
    maxLabel = []

    for i in range(df.shape[0]):
        maxLabel.append(Counter(df["labelsConesSlam"][i]))
    list_len = [len(i) for i in maxLabel]
    maxCols = []
    for i in range(max(list_len)):
        maxCol = 0
        for j in range(len(maxLabel)):
            if len(maxLabel[j]) > i:
                maxCol = max(maxLabel[j][f"{i}"], maxCol)

        maxCols.append(maxCol)

    labelsAndData = {}

    # create a dictionary every element is cone label [0] , [1] inside there is a list of the simulations and in that there is a list of the values of each label
    # {'0': [[...], [...]], '1': [[...],[...]]}
    for j in range(len(df["labelsConesSlam"])):
        labels = df["labelsConesSlam"][j]
        dataDistance = df["avgDistanceToConeSLAM"][j]
        for i in range(len(df["labelsConesSlam"][j])):
            label = labels[i]
            if label not in labelsAndData:
                labelsAndData[label] = []
                for _ in range(len(df["labelsConesSlam"])):
                    labelsAndData[label].append([])
            labelsAndData[label][j].append(dataDistance[i])

    cols = {}
    for idx, item in enumerate(labelsAndData.items()):
        for i in range(maxCols[idx]):
            cols[f"cone:{idx}-{i}"] = []
            for _, j in enumerate(item[1]):
                if len(j) - 1 >= i:
                    cols[f"cone:{idx}-{i}"].append(j[i])
                else:
                    cols[f"cone:{idx}-{i}"].append(np.nan)

    return pd.DataFrame(cols), df


def eval(newDF, df):
    data = {}
    for i in df["Parameters"]:
        for k in i:
            for item in k.items():
                if item[0] not in data:
                    data[item[0]] = []
                data[item[0]].append(item[1])
    otherDF = pd.DataFrame(data)

    otherDF["averageDist"] = newDF.mean(axis=1)
    otherDF["StopReason"] = df["StopReason"]
    otherDF = otherDF.loc[otherDF["StopReason"] == "asfinished"]
    return otherDF


def sendToFile(otherDF):
    printing = otherDF.drop(columns=["StopReason"])
    printing = printing.reset_index()
    printing = printing.drop(columns=["index"])

    out = "Run from today\n"
    for j in range(len(printing["averageDist"])):
        out += str(j) + ";"
        for i in printing:
            out += str(printing[i][j]) + ";"
        out += "\n"
    # Slack webhook URL
    slack_webhook_url = "https://hooks.slack.com/services/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"

    # Message to send
    message_payload = {"text": out}

    # Convert the dictionary to a JSON string
    json_payload = json.dumps(message_payload)

    # Make the POST request to Slack
    response = requests.post(
        slack_webhook_url,
        headers={"Content-Type": "application/json"},
        data=json_payload,
    )

    # Print the response from Slack
    print(response.text)


newDF, df = readFromFile()
otherDF = eval(newDF, df)
sendToFile(otherDF)
