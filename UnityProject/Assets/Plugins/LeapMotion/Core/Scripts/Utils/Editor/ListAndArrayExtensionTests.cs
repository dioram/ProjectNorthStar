/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

namespace Leap.Unity.Tests
{

    public class ListAndArrayExtensionTests
    {

        [Test]
        public void RemoveAtMany_Random()
        {
            List<int> toRemove = new List<int>().FillEach(100, i => i);
            while (toRemove.Count != 20)
            {
                toRemove.RemoveAt(Random.Range(0, toRemove.Count));
            }

            doRemoveAtManyTest(toRemove);
        }

        [Test]
        public void RemoveAtMany_First()
        {
            List<int> toRemove = new List<int>();
            toRemove.Add(0);
            toRemove.Add(50);

            doRemoveAtManyTest(toRemove);
        }

        [Test]
        public void RemoveAtMany_Last()
        {
            List<int> toRemove = new List<int>();
            toRemove.Add(50);
            toRemove.Add(99);

            doRemoveAtManyTest(toRemove);
        }

        [Test]
        public void RemoveAtMany_Sequential()
        {
            List<int> toRemove = new List<int>();
            toRemove.Add(50);
            toRemove.Add(51);
            toRemove.Add(52);
            toRemove.Add(53);

            doRemoveAtManyTest(toRemove);
        }

        [Test]
        public void InsertMany_Random()
        {
            List<int> toInsert = new List<int>().FillEach(20, i => i * 1000 + 99);
            List<int> indexes = new List<int>().FillEach(100, i => i);
            while (indexes.Count != toInsert.Count)
            {
                indexes.RemoveAt(Random.Range(0, indexes.Count));
            }

            doInsertManyTest(toInsert, indexes);
        }

        [Test]
        public void InsertMany_First()
        {
            List<int> toInsert = new List<int>();
            List<int> indexes = new List<int>();
            toInsert.Add(999);
            toInsert.Add(888);
            indexes.Add(0);
            indexes.Add(50);
            doInsertManyTest(toInsert, indexes);
        }

        [Test]
        public void InsertMany_Last()
        {
            List<int> toInsert = new List<int>();
            List<int> indexes = new List<int>();
            toInsert.Add(999);
            toInsert.Add(888);
            indexes.Add(50);
            indexes.Add(99);
            doInsertManyTest(toInsert, indexes);
        }

        [Test]
        public void InsertMany_Sequential()
        {
            List<int> toInsert = new List<int>();
            List<int> indexes = new List<int>();
            toInsert.Add(999);
            toInsert.Add(888);
            toInsert.Add(777);
            toInsert.Add(666);
            indexes.Add(50);
            indexes.Add(51);
            indexes.Add(52);
            indexes.Add(53);
            doInsertManyTest(toInsert, indexes);
        }

        private void doRemoveAtManyTest(List<int> toRemove)
        {
            List<int> listA = new List<int>().FillEach(100, i => i);
            List<int> listB = new List<int>(listA);

            for (int i = toRemove.Count; i-- != 0;)
            {
                listA.RemoveAt(toRemove[i]);
            }

            listB.RemoveAtMany(toRemove);

            Assert.AreEqual(listA, listB);
        }

        private void doInsertManyTest(List<int> toInsert, List<int> indexes)
        {
            List<int> listA = new List<int>().FillEach(100, i => i);
            List<int> listB = new List<int>(listA);

            for (int i = 0; i < toInsert.Count; i++)
            {
                listA.Insert(indexes[i], toInsert[i]);
            }

            listB.InsertMany(indexes, toInsert);

            Assert.AreEqual(listA, listB);
        }
    }
}
