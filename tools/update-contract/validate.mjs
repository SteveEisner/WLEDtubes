#!/usr/bin/env node
import {loadContract, validateContract} from './update-contract.mjs';

const errors = validateContract(loadContract());
if (errors.length) {
  for (const error of errors) console.error(error);
  process.exit(1);
}
console.log('Update contract is valid');
