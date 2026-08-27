const test = require('node:test');
const assert = require('node:assert/strict');

function rows(localId, peers, now) {
  return peers.filter(p => p.nodeId !== localId && now - p.lastSeenMs <= 60000)
    .sort((a,b) => Number(b.rssiKnown)-Number(a.rssiKnown)
      || (b.latestRssi ?? -127)-(a.latestRssi ?? -127) || a.nodeId-b.nodeId).slice(0,7);
}

test('fresh peers sort known RSSI strongest first, unknown last, bounded to seven', () => {
  const peers = [
    {nodeId:1,lastSeenMs:1000,rssiKnown:false},
    {nodeId:3,lastSeenMs:1000,rssiKnown:true,latestRssi:-40},
    {nodeId:2,lastSeenMs:1000,rssiKnown:true,latestRssi:-40},
    {nodeId:4,lastSeenMs:1000,rssiKnown:true,latestRssi:-80},
  ];
  assert.deepEqual(rows(9, peers, 1000).map(p=>p.nodeId), [2,3,4,1]);
  assert.equal(rows(99, Array.from({length:12},(_,i)=>({nodeId:i+1,lastSeenMs:1,rssiKnown:true,latestRssi:-i})), 1).length, 7);
  assert.deepEqual(rows(9, [{nodeId:9,lastSeenMs:999,rssiKnown:true,latestRssi:-1},{nodeId:8,lastSeenMs:1,rssiKnown:true,latestRssi:-1}], 61002), []);
});
