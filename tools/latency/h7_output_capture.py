#!/usr/bin/env python3
"""Decode H7OUT001 batches from raw real2sim recordings; never opens a port.

CONTROL records contain gated requests and paired wheel feedback. CAN records
contain actual attempted bytes and HAL status; neither is measured shaft torque.
"""
from __future__ import annotations
import argparse
import binascii
import csv
import hashlib
import json
from pathlib import Path
import struct

MAGIC = b'H7OUT001'
HEADER = struct.Struct('<8s4H6I')
RECORD = struct.Struct('<IIQQII6f2f2I4hH6B')
MAX_RECORDS = 48  # Accept both original 24-record and larger throughput batches.
MAX_PACKET = HEADER.size + MAX_RECORDS*RECORD.size + 2
assert HEADER.size == 40 and RECORD.size == 88


class CaptureParser:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = self.accepted = 0
        self.discarded_bytes = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets=[]
        while True:
            start=self.buffer.find(MAGIC)
            if start<0:
                discard=max(0,len(self.buffer)-len(MAGIC)+1)
                self.discarded_bytes+=discard;del self.buffer[:discard]
                break
            self.discarded_bytes+=start;del self.buffer[:start]
            if len(self.buffer)<HEADER.size:break
            header=HEADER.unpack_from(self.buffer)
            _,version,size,count,reserved,packet_seq,produced,dropped,queued,highwater,last=header
            if version!=1 or size!=RECORD.size or count>MAX_RECORDS or reserved or last or queued<count or highwater>128 or queued>128:
                self.rejected+=1;del self.buffer[0];continue
            length=HEADER.size+count*size+2
            if len(self.buffer)<length:break
            raw=bytes(self.buffer[:length])
            if binascii.crc_hqx(raw[:-2],0xffff)!=struct.unpack_from('<H',raw,length-2)[0]:
                self.rejected+=1;del self.buffer[0];continue
            records=[]
            for i in range(count):
                row=RECORD.unpack_from(raw,HEADER.size+i*size)
                if row[1] not in (1,2) or row[-3] not in (0,1) or row[-1]!=0:
                    raise ValueError('unsupported record type, machine or controller')
                records.append(row)
            packets.append({'sequence':packet_seq,'produced':produced,'dropped':dropped,
                            'queued':queued,'high_water':highwater,'records':records})
            self.accepted+=1;del self.buffer[:length]
        return packets


def analyze_session(session, out):
    # Reuse existing hash/event validation; no serial, physics or policy is run.
    import timing_session as mirror
    meta,_,_=mirror.load_session(session)
    parser=CaptureParser();records=[];packets=[];connections=0;fragments=[]
    for event in mirror.iter_session_events(session):
        if event['type']=='data':
            for packet in parser.feed(bytes.fromhex(event['data_hex'])):
                packet['receive_ns']=event['t_ns'];packet['connection']=connections
                for row in packet['records']:records.append((connections,event['t_ns'],row))
                packets.append(packet)
        elif event['type']=='connected':
            connections+=1
            if parser.buffer and (parser.buffer.startswith(MAGIC) or MAGIC.startswith(parser.buffer)):fragments.append(len(parser.buffer))
            parser.buffer.clear()
        elif event['type'] in ('disconnected','stop'):
            if parser.buffer and (parser.buffer.startswith(MAGIC) or MAGIC.startswith(parser.buffer)):fragments.append(len(parser.buffer))
            parser.buffer.clear()
    if not packets:raise ValueError('no H7OUT001 packets: old firmware has no complete-output capture')
    rows=[];gaps=[];restarts=0;last_seq=last_conn=None;intervals=[];last_end={}
    for conn,receive,row in records:
        seq,kind,begin,end,identifier,fault=row[:6]
        if end<begin:raise ValueError('record end precedes start')
        if last_seq is not None:
            delta=(seq-last_seq)&0xffffffff
            if conn!=last_conn or delta==0 or delta>=2**31:
                restarts+=1;last_end={}
            elif delta!=1:gaps.append({'after':last_seq,'before':seq,'missing':delta-1})
        if kind in last_end and begin>=last_end[kind]:intervals.append({'kind':kind,'period_us':(begin-last_end[kind])/1000})
        last_end[kind]=begin;last_seq,last_conn=seq,conn
        v={'connection':conn,'receive_ns':receive,'sequence':seq,'kind':'control' if kind==1 else 'can',
           'begin_ns':begin,'end_ns':end,'id':identifier,'fault':fault if kind==1 else '',
           'flags_or_hal':row[20],'strategy':row[21] if kind==1 else '',
           'bench_phase':row[22] if kind==1 else '', 'bench_reason':row[23] if kind==1 else '',
           'machine':row[24],'wheel_online':row[25] if kind==1 else '', 'controller_mode':row[26]}
        for j in range(6):v[f'request_{j}_firmware_nm']=row[6+j] if kind==1 else ''
        for j in range(2):
            v[f'wheel_{j}_speed_rad_s']=row[12+j] if kind==1 else ''
            v[f'wheel_{j}_rx_age_us']=row[14+j] if kind==1 and row[14+j]!=0xffffffff else ''
        for j in range(4):v[f'can_current_slot_{j}']=row[16+j] if kind==2 else ''
        rows.append(v)
    packet_gaps=[];drop_increase=0
    for previous,current in zip(packets,packets[1:]):
        if previous['connection']!=current['connection']:continue
        delta=(current['sequence']-previous['sequence'])&0xffffffff
        if 0<delta<2**31 and delta!=1:packet_gaps.append(delta-1)
        drops=(current['dropped']-previous['dropped'])&0xffffffff
        if drops<2**31:drop_increase+=drops
    complete=bool(records) and not (parser.rejected or gaps or restarts or packet_gaps or fragments or drop_increase)
    complete = complete and meta.get("capture_complete", False) and packets[-1]["queued"] == len(packets[-1]["records"])
    period={}
    for kind,label in ((1,'control'),(2,'can')):
        values=[x['period_us'] for x in intervals if x['kind']==kind]
        period[label]={'count':len(values),'min_us':min(values) if values else None,
                       'max_us':max(values) if values else None,'mean_us':sum(values)/len(values) if values else None}
    report={'format':'h7-output-capture-analysis','complete_observed_span':complete,
            'session':str(session),'stream_sha256':meta['stream_sha256'],'model':meta['model'],
            'machine_config':meta['machine_config'],'firmware_expectation':meta.get('firmware_manifest'),
            'packets':len(packets),'records':len(records),'crc_or_schema_rejections':parser.rejected,
            'event_sequence_gaps':gaps,'packet_sequence_gaps':packet_gaps,'restarts_or_reconnections':restarts,
            'queue_drop_increase':drop_increase,'first_total_queue_drops':packets[0]['dropped'],
            'last_total_queue_drops':packets[-1]['dropped'],'partial_packets_at_connection_end':fragments,
            'initial_queued_records_not_in_first_packet':packets[0]['queued']-len(packets[0]['records']),
            'last_queued_records_not_exported_yet':packets[-1]['queued']-len(packets[-1]['records']),
            'periods':period,'failed_CAN_attempts':sum(r['kind']=='can' and r['flags_or_hal']!=0 for r in rows),
            'scope':'completeness of the observed exported span; not CAN delivery, control deadline or physical acceptance',
            'limitations':['CAN begin/end bracket a HAL enqueue call, not electrical/motor application time',
                           'interleaved callers can overlap; queue order does not prove physical send order',
                           'pre-recording and post-recording outputs are outside the observed span',
                           'firmware manifest is an expected build supplied by operator, not boot attestation',
                           'request and raw current are not independently measured shaft torque']}
    out.mkdir(parents=True,exist_ok=False)
    if rows:
        with (out/'outputs.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (out/'packets.jsonl').open('w') as f:
        for packet in packets:f.write(json.dumps({k:v for k,v in packet.items() if k!='records'})+'\n')
    report['decoder_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    report=analyze_session(args.session,args.out)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['complete_observed_span']:raise SystemExit(2)
if __name__=='__main__':main()
