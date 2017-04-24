import random

def getPrimes(N):
    primes = []
    with open('prime_numbers.txt','r') as p:
        for line in p.readlines():
            for n in line.split():
                primes.append(int(n))
    return primes[0:N]

def getNRandomPrimes(allowedinds,num):
    newinds = random.sample(allowedinds,num)
    primes = getPrimes(max(allowedinds)+1)
    return [primes[i] for i in newinds]
